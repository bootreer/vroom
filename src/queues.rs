use crate::cmd::NvmeCommand;
use crate::memory::*;
use std::error::Error;
use std::hint::spin_loop;

/// NVMe spec 4.6
/// Completion queue entry
#[allow(dead_code)]
#[derive(Clone, Copy, Debug, Default)]
#[repr(C, packed)]
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

/// maximum amount of submission entries on a 2MiB huge page
pub const QUEUE_LENGTH: usize = 1024;

/// Submission queue
pub struct NvmeSubQueue {
    // TODO: switch to mempool for larger queue
    commands: Dma<[NvmeCommand; QUEUE_LENGTH]>,
    pub head: usize,
    pub tail: usize,
    len: usize,
    pub doorbell: usize,
}

impl NvmeSubQueue {
    pub fn new(len: usize, doorbell: usize) -> Result<Self, Box<dyn Error>> {
        Ok(Self {
            commands: Dma::allocate(crate::memory::HUGE_PAGE_SIZE)?,
            head: 0,
            tail: 0,
            len: len.min(QUEUE_LENGTH),
            doorbell,
        })
    }

    pub fn is_empty(&self) -> bool {
        self.head == self.tail
    }

    pub fn is_full(&self) -> bool {
        self.head == (self.tail + 1) % self.len
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
        self.commands[self.tail] = entry;

        self.tail = (self.tail + 1) % self.len;
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
    len: usize,
    pub doorbell: usize,
}

// TODO: error handling
impl NvmeCompQueue {
    pub fn new(len: usize, doorbell: usize) -> Result<Self, Box<dyn Error>> {
        Ok(Self {
            commands: Dma::allocate(crate::memory::HUGE_PAGE_SIZE)?,
            head: 0,
            phase: true,
            len: len.min(QUEUE_LENGTH),
            doorbell,
        })
    }

    #[inline(always)]
    pub fn complete(&mut self) -> Option<(usize, NvmeCompletion, usize)> {
        let entry = &self.commands[self.head];

        if ((entry.status & 1) == 1) == self.phase {
            let prev = self.head;
            self.head = (self.head + 1) % self.len;
            if self.head == 0 {
                self.phase = !self.phase;
            }
            Some((self.head, entry.clone(), prev))
        } else {
            None
        }
    }

    ///
    #[inline(always)]
    pub fn complete_n(&mut self, commands: usize) -> (usize, NvmeCompletion, usize) {
        let prev = self.head;
        self.head += commands - 1;
        if self.head >= self.len {
            self.phase = !self.phase;
        }
        self.head %= self.len;

        let (head, entry, _) = self.complete_spin();
        (head, entry, prev)
    }

    #[inline(always)]
    pub fn complete_spin(&mut self) -> (usize, NvmeCompletion, usize) {
        loop {
            if let Some(val) = self.complete() {
                return val;
            }
            spin_loop();
        }
    }

    pub fn get_addr(&self) -> usize {
        self.commands.phys
    }
}
