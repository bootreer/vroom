use crate::queues::*;

pub struct NvmeDevice {
    comp_queues: Vec<NvmeCompQueue>,
    sub_queues: Vec<NvmeSubQueue>,
}
