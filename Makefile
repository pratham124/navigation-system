run:
	gnome-terminal -- python3 client/client.py
	./server/server

clean:
	rm -f inpipe
	rm -f outpipe
	rm -f server/server
	rm -f server/digraph.o
	rm -f server/dijkstra.o
	rm -f server/server.o