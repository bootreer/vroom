# vroom
vroom is a userspace NVMe driver written in Rust. It aims to be as fast as the SPDK NVMe driver, while minimizing unsafe code and offering a simplified API. vroom currently serves as a proof of concept. 

[My thesis](https://db.in.tum.de/people/sites/ellmann/theses/finished/24/pirhonen_writing_an_nvme_driver_in_rust.pdf) contains some details about the implementation.

# Build instructions
You will need Rust, as well as its package manager `cargo` which you can install with:
```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

Huge pages need to be enabled:
```bash
cd vroom
sudo ./setup-hugetlbfs.sh
```

To build the driver, as well as any examples run:
```bash
cargo build --release --all-targets
```

e.g. to run the hello world example (root rights are needed for DMA):
```
sudo ./target/release/examples/hello_world 0000:00:07.0
```

# Disclaimer
This is by no means production-ready. Do not use it in critical environments. DMA may corrupt memory.

# Related projects
- [Redox's NVMe driver](https://gitlab.redox-os.org/redox-os/drivers/-/tree/master/storage/nvmed)
