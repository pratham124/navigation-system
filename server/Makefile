CC=g++
PROGS= server
OBJS= server.o dijkstra.o digraph.o


server: server.o dijkstra.o digraph.o
    $(CC) -o server server.o dijkstra.o digraph.o


server.o: server.cpp
    $(CC) -c server.cpp

dijkstra.o: dijkstra.cpp dijkstra.h
    $(CC) -c dijkstra.cpp

digraph.o: digraph.cpp digraph.h wdigraph.h
    $(CC) -c digraph.cpp

clean:
    @rm $(OBJS) $(PROGS)