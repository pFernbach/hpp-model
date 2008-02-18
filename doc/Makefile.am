# 
#  Copyright 
# 


hppModeldir = $(datadir)/doc/hppModel

EXTRA_DIST = \
	header.html \
	footer.html \
	package.css \
	pictures/footer.jpg \
	figures/classDiagramRobot.dot \
	figures/classDiagramRobot.png \
	figures/classDiagramJoint.dot \
	figures/classDiagramJoint.png \
	figures/classDiagramBody.dot \
	figures/classDiagramBody.png \
	additionalDoc/package.h 


install: html-doc 
	mkdir -p $(hppModeldir)
	$(INSTALL_DATA) html/* $(hppModeldir)
	mkdir -p $(hppModeldir)/pictures
	$(INSTALL_DATA) $(srcdir)/pictures/* $(hppModeldir)/pictures
	mkdir -p $(datadir)/doc/doxytag
	$(INSTALL_DATA) hppModel.doxytag $(datadir)/doc/doxytag

html-doc: package.dox
	doxygen $<
