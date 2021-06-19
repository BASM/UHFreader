
CFLAGS=-ggdb3 -Wall -Werror -Wextra

all: test

test: libuhreader.a
	${CC} -o test -ggdb3 ${CFLAGS} main.c -lssl -lcrypto -I. $^

libuhreader.a: uhreader.o
	${AR} rvs libuhreader.a uhreader.o


uhreader.o: uhreader.c
	$(CC) -c -o $@ ${CFLAGS} $^

clean:
	rm -f libuhreader.a uhreader.o test

run: secret
	#sudo ./test -p 1122334455667788 -P 1234567801234567
	sudo ./test `cat secret` 

fix: secret
	sudo ./test `cat secret` -fv

wg:
	#100500
	sudo ./test -w0x18894

unlock: secret
	sudo ./test `cat secret` -uv

secret:
	echo "-p 1122334455667788 -P 1234567801234567" > secret

beep:
	sudo ./test -b
