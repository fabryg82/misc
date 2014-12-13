CC=g++
CFLAGS=-c -g -Wall 
LDFLAGS=
SOURCES=app.cpp serial.cpp 
OBJECTS=$(SOURCES:.cpp=.o)
EXEC=exec

all: $(SOURCES)	$(EXEC)
	
$(EXEC): $(OBJECTS) 
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@

.c.o:
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm -rf *.o *.a *~ $(EXEC)

#install: $(EXEC)
#	[ -d $(INSTDIR)/usr/sbin ] || mkdir -p $(INSTDIR)/usr/sbin
#	install -c $(EXEC) $(INSTDIR)/usr/sbin

#uninstall:
#	rm $(INSTDIR)/usr/sbin/$(EXEC)



