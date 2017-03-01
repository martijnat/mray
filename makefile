PREFIX = /usr

all:
	cython mray.py --embed --verbose
	gcc -O3 -I/usr/include/python3.6m -lpython3.6m mray.c -o mray-bin

install:
	mkdir -p $(DESTDIR)$(PREFIX)/bin
	cp -p mray-bin $(DESTDIR)$(PREFIX)/bin/mray

uninstall:
	rm -f $(DESTDIR)$(PREFIX)/bin/mray

clean:
	rm mray.c
	rm mray-bin

