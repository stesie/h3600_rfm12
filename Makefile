
all:
	$(MAKE) -C $(TOPDIR) modules SUBDIRS=$(PWD)

clean:
	rm -f *.[osa] *~

obj-m		:= testmod.o

include $(TOPDIR)/Rules.make
