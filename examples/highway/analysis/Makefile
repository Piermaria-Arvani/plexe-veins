# tool for indexing vec files
SCAVETOOL = scavetool
# scripts location
SCRIPTDIR = .
# results location
RESDIR = ../results
# script for merging
MERGESCRIPT = $(SCRIPTDIR)/merge.R

# match all .vec files for the Highway config
HIGHWAY      = $(wildcard $(RESDIR)/Highway*.vec)
# change suffix from .vec to .Rdata and add the highway prefix
HIGHWAY_DATA = $(HIGHWAY:$(RESDIR)/%.vec=$(RESDIR)/highway.%.Rdata)

# vector index files and Rdata files
VCI = $(VECTOR:%.vec=%.vci)
RDATA = $(VECTOR:%.vec=%.Rdata)

# all make targets
all: Highway.Rdata 

# to make Highway.Rdata we need to merge all files starting with highway.Highway
# before this, check that all HIGHWAY_DATA files have been processed
$(RESDIR)/Highway.Rdata: $(HIGHWAY_DATA)
	Rscript $(MERGESCRIPT) $(RESDIR)/ highway.Highway $(notdir $@) map-config default
Highway.Rdata: $(RESDIR)/Highway.Rdata

# to make all highway.*.Rdata files we need to run the generic parser
highway.%.Rdata: %.vec %.vci
	Rscript generic-parser.R $< map-config default highway

# if vec files are not indexed, index them
%.vci : %.vec
	$(SCAVETOOL) index $<

# helper to print variable values. e.g.: make print-DELAY_DATA
print-%:
	@echo '$*=$($*)'

# every intermediate file is kept instead of being automatically deleted.
# .vci files are detected as intermediate and thus cancelled when the make
# command terminates. however, such files can still be needed by other
# targets, so keep them instead of re-doing indexing
.SECONDARY :
