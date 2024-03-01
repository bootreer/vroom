use lazy_static::lazy_static;
use libc::munmap;
use libc::c_void;
use core::slice;
// use std::rc::Rc;
// use std::cell::RefCell;
use std::collections::HashMap;
use std::error::Error;
use std::io::{self, Read, Seek};
use std::os::fd::{AsRawFd, RawFd};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Mutex;
use std::{fs, mem, process, ptr};
use std::ops::{Deref, DerefMut, Index, IndexMut, Range, RangeTo};

// from https://www.kernel.org/doc/Documentation/x86/x86_64/mm.txt
const X86_VA_WIDTH: u8 = 47;

const HUGE_PAGE_BITS: u32 = 21;
pub const HUGE_PAGE_SIZE: usize = 1 << HUGE_PAGE_BITS;

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
    size: usize,
}

// should be safe
impl<T> Deref for Dma<T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        unsafe {
            &*self.virt
        }
    }
}

impl<T> DerefMut for Dma<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        unsafe {
            &mut *self.virt
        }
    }
}

// Trait for types that can be viewed as consecutive DMA slices.
pub trait DmaSlice {
    fn chunks(&self, chunk_size: usize) -> DmaChunks<u8>;
}

pub struct DmaChunks<'a, T> {
    current_offset: usize,
    chunk_size: usize,
    dma: &'a Dma<T>,
}

impl<'a, T> Iterator for DmaChunks<'a, T> {
    type Item = DmaChunk<'a, T>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.current_offset >= self.dma.size {
            None
        } else {
            let chunk_phys_addr = self.dma.phys + self.current_offset * std::mem::size_of::<T>();
            let offset_ptr = unsafe { self.dma.virt.add(self.current_offset) };
            let len = std::cmp::min(self.chunk_size, (self.dma.size - self.current_offset) / std::mem::size_of::<T>());

            self.current_offset += len;

            Some(DmaChunk {
                phys_addr: chunk_phys_addr,
                slice: unsafe { std::slice::from_raw_parts_mut(offset_ptr, len) },
            })
        }
    }
}

impl Index<Range<usize>> for Dma<u8> {
    type Output = [u8];

    fn index(&self, index: Range<usize>) -> &Self::Output {
        assert!(index.end <= self.size, "Index out of bounds");

        unsafe {
            slice::from_raw_parts(self.virt.add(index.start), index.end - index.start)
        }
    }
}

impl IndexMut<Range<usize>> for Dma<u8> {
    fn index_mut(&mut self, index: Range<usize>) -> &mut Self::Output {
        assert!(index.end <= self.size, "Index out of bounds");
        unsafe {
            slice::from_raw_parts_mut(self.virt.add(index.start), index.end - index.start)
        }
    }
}

impl Index<RangeTo<usize>> for Dma<u8> {
    type Output = [u8];

    fn index(&self, index: RangeTo<usize>) -> &Self::Output {
        assert!(index.end <= self.size, "Index out of bounds");

        unsafe {
            slice::from_raw_parts(self.virt, index.end)
        }
    }
}

impl IndexMut<RangeTo<usize>> for Dma<u8> {
    fn index_mut(&mut self, index: RangeTo<usize>) -> &mut Self::Output {
        assert!(index.end <= self.size, "Index out of bounds");
        unsafe {
            slice::from_raw_parts_mut(self.virt, index.end)
        }
    }
}

// Represents a chunk obtained from a Dma<T>, with physical address and slice.
pub struct DmaChunk<'a, T> {
    pub phys_addr: usize,
    pub slice: &'a mut [T],
}

impl DmaSlice for Dma<u8> {
    fn chunks(&self, chunk_size: usize) -> DmaChunks<u8> {
        DmaChunks {
            current_offset: 0,
            chunk_size,
            dma: self,
        }
    }
}

impl<T> Dma<T> {
    /// Allocates DMA Memory on a huge page
    // TODO: vfio support?
    pub fn allocate(size: usize, require_contiguous: bool) -> Result<Dma<T>, Box<dyn Error>> {
        let size = if size % HUGE_PAGE_SIZE != 0 {
            ((size >> HUGE_PAGE_BITS) + 1) << HUGE_PAGE_BITS
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
                        // libc::MAP_SHARED,
                        f.as_raw_fd(),
                        0,
                    )
                };
                if ptr == libc::MAP_FAILED {
                    Err("failed to mmap huge page - are huge pages enabled and free?".into())
                } else if unsafe { libc::mlock(ptr, size) } == 0 {
                    let memory = Dma {
                        // virt: NonNull::new(ptr as *mut T).expect("oops"),
                        virt: ptr as *mut T,
                        phys: virt_to_phys(ptr as usize)?,
                        size
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

// idk if required
impl<T> Drop for Dma<T> {
    fn drop(&mut self) {
        unsafe {
            // munmap(self.virt.as_ptr() as *mut c_void, self.size);
            munmap(self.virt as *mut c_void, self.size);
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
    Ok((phys & 0x007F_FFFF_FFFF_FFFF) * pagesize + addr % pagesize)
}

#[allow(unused)]
pub fn vfio_enabled() -> bool {
    unsafe { VFIO_CONTAINER_FILE_DESCRIPTOR.is_some() }
}
