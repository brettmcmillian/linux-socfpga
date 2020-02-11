#
# Makefile for the Altera device drivers.
#

obj-$(CONFIG_ALTERA_TSE) += altera_tse.o
altera_tse-objs := altera_tse_main.o altera_tse_ethtool.o \
altera_msgdma.o altera_sgdma.o altera_utils.o

obj-$(CONFIG_INTEL_S10_100GHIP) += intel_s10_100ghip.o
altera_s10_100ghip-objs := altera_s10_100ghip.o \
altera_s10_msgdma.o altera_s10_utils.o
