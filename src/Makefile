SOURCES = $(wildcard *.cpp)
OBJECTS = $(SOURCES:.cpp=.o)
DEPENDS = $(SOURCES:.cpp=.d) 
LDFLAGS = $(shell pkg-config --libs lua5.1) -llua5.1 -lpng  -DQT_CORE_LIB  -lpthread  -framework QtGui -framework QtCore -F/usr/local/Cellar/qt5/5.4.0/lib
INCPATH = -I/usr/include/qt5/QtGui -I/usr/include/qt5 -I/usr/include/qt5/QtCore -I/usr/include/lua5.1 -I/usr/local/Cellar/lua51/5.1.5/include/lua-5.1 -I/usr/local/Cellar/qt5/5.4.0/lib/QtGui.framework/Versions/5/Headers -I/usr/local/Cellar/qt5/5.4.0/lib/QtCore.framework/Versions/5/Headers -F/usr/local/Cellar/qt5/5.4.0/lib
CPPFLAGS = $(shell pkg-config --cflags lua5.1) -fPIE -std=c++11 -O3
CXXFLAGS = $(CPPFLAGS) -W -Wall -g -fPIE -O3 -ggdb
CXX = g++
MAIN = rt

all: $(MAIN)

depend: $(DEPENDS)

clean:
	rm -f *.o *.d $(MAIN)

$(MAIN): $(OBJECTS)
	@echo Creating $@...
	@$(CXX) -o $@ $(OBJECTS) $(LDFLAGS)

%.o: %.cpp
	@echo Compiling $<...
	@$(CXX) -o $@ -c $(CXXFLAGS) $(INCPATH) $<

%.d: %.cpp
	@echo Building $@...
	@set -e; $(CC) -M $(CPPFLAGS) $(INCPATH) $< \
                  | sed 's/\($*\)\.o[ :]*/\1.o $@ : /g' > $@; \
                [ -s $@ ] || rm -f $@

include $(DEPENDS)
