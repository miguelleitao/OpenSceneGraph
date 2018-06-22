
INC_BULLET=-I/usr/include/bullet
INC_OSG=-I ../include
LD_OSG=-L ../lib -losg -losgViewer -losgSim -l osgDB -l osgGA
LD_BULLET=-l BulletDynamics -l BulletCollision -l LinearMath
BASE_DIR=http://ave.dee.isep.ipp.pt/~jml/intmu/lab9

bola: bola.o
	cc -O2 -o $@ $^ ${LD_OSG} ${LD_BULLET} -l stdc++ -lm

bola.o: bola.cpp
	g++  -Wall -O2  -c ${INC_OSG} ${INC_BULLET} $<
	
bullet_hello: bullet_hello.o
	cc -O2 -o $@ $^ -l BulletDynamics -l BulletCollision -l LinearMath -l stdc++

bola_bullet: bola_bullet.o
	cc -O2 -o $@ $^ -l BulletDynamics -l BulletCollision -l LinearMath ${LD_OSG} -l stdc++ -lm

bullet_hello.o: bullet_hello.cpp
	cc -Wall -O2 -c ${INC_BULLET} $<

bola_bullet.o: bola_bullet.cpp
	g++ -Wall -O2 -c ${INC_BULLET} ${INC_OSG} $<

run: bola
	LD_LIBRARY_PATH=../lib ./bola

clean:
	rm -f bullet_hello.o bola_bullet.o bola bola_bullet bullet_hello bola.o

.PHONY: dist
dist: clean
	rm -f bola.cpp bullet_hello.cpp bola.mtl bola.obj plano.mtl plano.obj
	make get

