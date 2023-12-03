use lazy_static::lazy_static;
use libc::{c_void, memset};
use std::cell::RefCell;
use std::collections::HashMap;
use std::error::Error;
use std::io::{self, Read, Seek};
use std::os::fd::{AsRawFd, RawFd};
use std::rc::Rc;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Mutex;
use std::{fs, mem, process, ptr};

// from https://www.kernel.org/doc/Documentation/x86/x86_64/mm.txt
const X86_VA_WIDTH: u8 = 47;

const HUGE_PAGE_BITS: u32 = 21;
const HUGE_PAGE_SIZE: usize = 1 << HUGE_PAGE_BITS;

pub const IOVA_WIDTH: u8 = X86_VA_WIDTH;

static HUGEPAGE_ID: AtomicUsize = AtomicUsize::new(0);

pub(crate) static mut VFIO_CONTAINER_FILE_DESCRIPTOR: Option<RawFd> = None;

lazy_static! {
    pub(crate) static ref VFIO_GROUP_FILE_DESCRIPTORS: Mutex<HashMap<i32, RawFd>> =
        Mutex::new(HashMap::new());
}

pub struct Dma<T> {
    pub virt: *mut T,
    pub phys: usize,
}

impl<T> Dma<T> {
    /// Allocates DMA Memory on a huge page
    // TODO: vfio support?
    #[allow(arithmetic_overflow)]
    pub fn allocate(size: usize, require_contiguous: bool) -> Result<Dma<T>, Box<dyn Error>> {
        let size = if size % HUGE_PAGE_SIZE != 0 {
            ((size >> HUGE_PAGE_BITS) + 1) << HUGE_PAGE_SIZE // sus
        } else {
            size
        };

        if require_contiguous && size > HUGE_PAGE_SIZE {
            return Err("failed to map physically contiguous memory".into());
        }

        let id = HUGEPAGE_ID.fetch_add(1, Ordering::SeqCst);
        let path = format!("/mnt/huge/nvme-{}-{}", process::id(), id);

        match fs::OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .open(path.clone())
        {
            Ok(f) => {
                let ptr = unsafe {
                    libc::mmap(
                        ptr::null_mut(),
                        size,
                        libc::PROT_READ | libc::PROT_WRITE,
                        libc::MAP_SHARED | libc::MAP_HUGETLB,
                        // libc::MAP_SHARED, // cuz MAP_HUGETLB doesn't exist on macOS (for lsp lol)
                        f.as_raw_fd(),
                        0,
                    )
                };
                if ptr == libc::MAP_FAILED {
                    Err("failed to mmap huge page - are huge pages enabled and free?".into())
                } else if unsafe { libc::mlock(ptr as *mut libc::c_void, size) } == 0 {
                    let memory = Dma {
                        virt: ptr as *mut T,
                        phys: virt_to_phys(ptr as usize)?,
                    };
                    Ok(memory)
                } else {
                    Err("failed to memory lock huge page".into())
                }
            }
            Err(ref e) if e.kind() == io::ErrorKind::NotFound => Err(Box::new(io::Error::new(
                e.kind(),
                format!(
                    "huge page {} could not be created - huge pages enabled?",
                    path
                ),
            ))),
            Err(e) => Err(Box::new(e)),
        }
    }
}

pub struct Mempool {
    base_addr: *mut u8,
    num_entries: usize,
    entry_size: usize,
    phys_addresses: Vec<usize>,
    pub(crate) free_stack: RefCell<Vec<usize>>,
}

impl Mempool {
    /// Allocates a new `Mempool`.
    ///
    /// # Panics
    ///
    /// Panics if `size` is not a divisor of the page size.
    pub fn allocate(entries: usize, size: usize) -> Result<Rc<Mempool>, Box<dyn Error>> {
        let entry_size = match size {
            0 => 2048,
            x => x,
        };

        if HUGE_PAGE_SIZE % entry_size != 0 {
            panic!("entry size must be a divisor of the page size");
        }

        let dma: Dma<u8> = Dma::allocate(entries * entry_size, false)?;
        let mut phys_addresses = Vec::with_capacity(entries);

        for i in 0..entries {
            phys_addresses.push(unsafe { virt_to_phys(dma.virt.add(i * entry_size) as usize)? });
        }

        let pool = Mempool {
            base_addr: dma.virt,
            num_entries: entries,
            entry_size,
            phys_addresses,
            free_stack: RefCell::new(Vec::with_capacity(entries)),
        };

        unsafe {
            memset(
                pool.base_addr as *mut c_void,
                (pool.num_entries * pool.entry_size) as i32,
                0x00,
            )
        };

        let pool = Rc::new(pool);
        pool.free_stack.borrow_mut().extend(0..entries);

        Ok(pool)
    }
}

/// Translates a virtual address to its physical counterpart
pub(crate) fn virt_to_phys(addr: usize) -> Result<usize, Box<dyn Error>> {
    let pagesize = unsafe { libc::sysconf(libc::_SC_PAGESIZE) } as usize;

    let mut file = fs::OpenOptions::new()
        .read(true)
        .open("/proc/self/pagemap")?;

    file.seek(io::SeekFrom::Start(
        (addr / pagesize * mem::size_of::<usize>()) as u64,
    ))?;

    let mut buffer = [0; mem::size_of::<usize>()];
    file.read_exact(&mut buffer)?;

    let phys = unsafe { mem::transmute::<[u8; mem::size_of::<usize>()], usize>(buffer) };
    Ok((phys & 0x007F_FFFF_FFFF_FFFF) * pagesize + addr & pagesize)
}

#[allow(unused)]
pub fn vfio_enabled() -> bool {
    unsafe { VFIO_CONTAINER_FILE_DESCRIPTOR != None }
}
