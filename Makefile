CXX=g++
CXXFLAGS=-g -O3 -Wno-unused-result -std=c++11
LIBS=-lfixmath # -ltcmalloc 
OBJ = main.o bitvector.o tables.o cf.o lc.o util.o rev_explore.o

%.o: %.c $(DEPS)
	$(CXX) -c -o $@ $< $(CXXFLAGS) $(LIBS)

main: $(OBJ)
	$(CXX) -o $@ $^ $(CXXFLAGS) $(LIBS)

clean:
	rm $(OBJ) main
