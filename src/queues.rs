use std::error::Error;
use crate::cmd::NvmeCommand;
use crate::memory::*;

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

const QUEUE_LENGTH: usize = 1024;

/// Submission queue
#[allow(dead_code)]
pub struct NvmeSubQueue {
    commands: Dma<[NvmeCommand; QUEUE_LENGTH]>,
    head: usize,
    tail: usize,
}

impl NvmeSubQueue {
    pub fn new() -> Result<Self, Box<dyn Error>> {
        Ok(Self {
            commands: Dma::allocate(64 * QUEUE_LENGTH, true)?,
            head: 0,
            tail: 0,
        })
    }

    pub fn is_empty(&self) -> bool {
        self.head == self.tail
    }
    pub fn is_full(&self) -> bool {
        self.head == self.tail + 1
    }

    #[allow(unused)]
    pub fn submit(&mut self, entry: NvmeCommand) -> usize {
        // TODO
        unsafe { // seems legit
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
#[allow(dead_code)]
pub struct NvmeCompQueue {
    commands: Dma<[NvmeCommand; QUEUE_LENGTH]>,
    head: usize,
    phase: bool,
}

impl NvmeCompQueue {
    pub fn new() -> Result<Self, Box<dyn Error>> {
        Ok(Self {
            commands: Dma::allocate(64 * QUEUE_LENGTH, true)?,
            head: 0,
            phase: true,
        })
    }

    pub fn get_addr(&self) -> usize {
        self.commands.phys
    }
}
