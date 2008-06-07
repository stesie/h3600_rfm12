
all:
	$(MAKE) -C $(TOPDIR) modules SUBDIRS=$(PWD)

clean:
	rm -f *.[osa] *~

obj-m		:= h3600_rfm12.o

include $(TOPDIR)/Rules.make
