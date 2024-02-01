use crate::cmd::NvmeCommand;
use crate::memory::*;
use std::error::Error;

static mut PRINT: bool = true;

/// NVMe spec 4.6
/// Completion queue entry
#[allow(dead_code)]
#[derive(Clone, Copy, Debug, Default)]
#[repr(packed)]
pub struct NvmeCompletion {
    /// Command specific
    pub command_specific: u32,
    /// Reserved
    pub _rsvd: u32,
    // Submission queue head
    pub sq_head: u16,
    // Submission queue ID
    pub sq_id: u16,
    // Command ID
    pub c_id: u16,
    //  Status field
    pub status: u16,
}

pub const QUEUE_LENGTH: usize = 1024;

/// Submission queue
pub struct NvmeSubQueue {
    // TODO: switch to mempool for larger queue
    pub commands: Dma<[NvmeCommand; QUEUE_LENGTH]>,
    pub head: usize,
    pub tail: usize,
}

impl NvmeSubQueue {
    pub fn new() -> Result<Self, Box<dyn Error>> {
        // let commands: Dma<[NvmeCommand; QUEUE_LENGTH]> = Dma::allocate(64 * QUEUE_LENGTH, false)?;

        Ok(Self {
            // commands: Dma::allocate(64 * QUEUE_LENGTH, true)?,
            commands: Dma::allocate(crate::memory::HUGE_PAGE_SIZE, false)?,
            head: 0,
            tail: 0,
        })
    }

    pub fn is_empty(&self) -> bool {
        self.head == self.tail
    }

    pub fn is_full(&self) -> bool {
        self.head == (self.tail + 1) % QUEUE_LENGTH
    }

    pub fn submit_checked(&mut self, entry: NvmeCommand) -> Option<usize> {
        if self.is_full() {
            None
        } else {
            Some(self.submit(entry))
        }
    }

    #[inline(always)]
    pub fn submit(&mut self, entry: NvmeCommand) -> usize {
        // println!("SUBMISSION ENTRY: {:?}", entry);
        unsafe {
            (*self.commands.virt)[self.tail] = entry;
        }
        self.tail = (self.tail + 1) % QUEUE_LENGTH;
        self.tail
    }

    pub fn get_addr(&self) -> usize {
        self.commands.phys
    }
}

/// Completion queue
pub struct NvmeCompQueue {
    commands: Dma<[NvmeCompletion; QUEUE_LENGTH]>,
    head: usize,
    phase: bool,
}

// TODO: error handling
impl NvmeCompQueue {
    pub fn new() -> Result<Self, Box<dyn Error>> {
        Ok(Self {
            commands: Dma::allocate(crate::memory::HUGE_PAGE_SIZE, false)?,
            head: 0,
            phase: true,
        })
    }

    pub fn complete(&mut self) -> Option<(usize, NvmeCompletion, usize)> {
        let entry: NvmeCompletion = unsafe { (*self.commands.virt)[self.head] };

        if ((entry.status & 1) == 1) == self.phase {
            let prev = self.head;
            self.head = (self.head + 1) % QUEUE_LENGTH;
            if self.head == 0 {
                self.phase = !self.phase;
            }

            // println!("COMPLETION ENTRY: {:?}", entry);
            Some((self.head, entry, prev))
        } else {
            None
        }
    }

    ///
    pub fn complete_n(&mut self, commands: usize) -> (usize, NvmeCompletion, usize) {
        let prev = self.head;
        self.head += commands - 1;
        if self.head >= QUEUE_LENGTH {
            self.phase = !self.phase;
        }
        self.head %= QUEUE_LENGTH;

        let (head, entry, _) = self.complete_spin();
        (head, entry, prev)
    }

    pub fn complete_spin(&mut self) -> (usize, NvmeCompletion, usize) {
        loop {
            if let Some(val) = self.complete() {
                return val;
            } else {
                super::pause();
            }
        }
    }

    pub fn get_addr(&self) -> usize {
        self.commands.phys
    }
}
