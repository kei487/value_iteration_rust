.PHONY: driver host test-host test-hw clean

driver:
	$(MAKE) -C driver/uio all

host: driver
	$(MAKE) -C host all

test-host:
	$(MAKE) -C host test-host

test-hw:
	$(MAKE) -C host test-hw

clean:
	$(MAKE) -C driver/uio clean
	$(MAKE) -C host clean
