
#
# Makefile for the linux kernel.
#

# Object file lists.

obj-y			:= cpu.o irq.o time.o dev.o clock.o dmac.o fmi.o edma.o DrvEDMA.o
obj-m			:=
obj-n			:=
obj-			:=

obj-$(CONFIG_CPU_W55FA93)	+= w55fa93.o

# machine specific support
obj-$(CONFIG_ARCH_W55FA93)	+= mach-w55fa93.o

# Power Management
obj-$(CONFIG_CPU_FREQ)		+= cpufreq.o
obj-$(CONFIG_CPU_IDLE)		+= cpuidle.o
obj-$(CONFIG_SUSPEND)		+= pm.o

