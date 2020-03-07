run: csma.cpp
	g++ csma.cpp -o csma.o
	./csma.o

clean:
	rm csma.o
	rm Efficiency_persistent.csv
	rm Throughput_persistent.csv
	rm Efficiency_nonPersistent.csv
	rm Throughput_nonPersistent.csv
