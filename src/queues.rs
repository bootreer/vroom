
/// NVMe Spec 4.2
/// Submission queue entry
#[allow(dead_code)]
#[derive(Clone, Copy, Debug, Default)]
#[repr(packed)]
pub struct NvmeCommand {
    /// Opcode
    pub opcode: u8,
    /// Flags; FUSE (2 bits) | Reserved (4 bits) | PSDT (2 bits)
    pub flags: u8,
    /// Command ID
    pub c_id: u16,
    /// Namespace ID
    pub ns_id: u32,
    /// Reserved
    pub _rsvd: u64,
    /// Metadata pointer
    pub md_ptr: u64,
    /// Datapointer
    pub d_ptr: [u64; 2],
    /// Command dword 10
    pub cdw10: u32,
    /// Command dword 11
    pub cdw11: u32,
    /// Command dword 12
    pub cdw12: u32,
    /// Command dword 13
    pub cdw13: u32,
    /// Command dword 14
    pub cdw14: u32,
    /// Command dword 15
    pub cdw15: u32,
}

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

/// Submission queue
#[allow(dead_code)]
pub struct NvmeSubQueue {

}

/// Completion queue
#[allow(dead_code)]
pub struct NvmeCompQueue {

}
