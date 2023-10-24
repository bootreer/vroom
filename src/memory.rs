use std::error::Error;
use std::io::{self, Seek, Read};
use std::os::fd::AsRawFd;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::{fs, mem, process, ptr};

const HUGE_PAGE_BITS: u32 = 32;
const HUGE_PAGE_SIZE: usize = 1 << HUGE_PAGE_BITS;

static HUGEPAGE_ID: AtomicUsize = AtomicUsize::new(0);

pub struct Dma<T> {
    pub virt: *mut T,
    pub phys: usize,
}

impl<T> Dma<T> {
    /// Allocates DMA Memory on a huge page
    // TODO: does linux still not move pages of size 2MiB?
    pub fn allocate(size: usize, require_contiguous: bool) -> Result<Dma<T>, Box<dyn Error>> {
        let size = if size % HUGE_PAGE_SIZE != 0 {
            ((size >> HUGE_PAGE_BITS) + 1) << HUGE_PAGE_SIZE
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
                        // libc::MAP_SHARED | libc::MAP_HUGETLB,
                        libc::MAP_SHARED, // cuz MAP_HUGETLB doesn't exist on macOS
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
