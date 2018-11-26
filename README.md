# N32905_Linux_BSP
Linux BSP Source Code for N9H20/N32905 Microprocessor 

- **Extract rootfs tarball before build the krenel image** 
`$ tar zxvf rootfs-2.6.35.tar.gz`

- **Load N9H20 default kernel configuration**
`$ make n9h20k5_defconfig`

- **Load N32905 default kernel configuration**
`$ make w55fa93_defconfig`

- **Build kernel image** 
  `$ ./build`

- **Change Booting Device**
`$ ./build [sd/nand/spi]`

N9H2/N329 series BSP supports SD/NAND/SPI booting devices. You can type “./build [sd/nand/spi]” to build kernel with specified booting device.
For ex: `$ ./build sd`
