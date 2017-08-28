# tool for indexing vec files
SCAVETOOL = scavetool
# scripts location
SCRIPTDIR = .
# results location
RESDIR = ../results
# script for merging
MERGESCRIPT = $(SCRIPTDIR)/merge.R

# match all .vec files for the Braking config
BRAKING         = $(wildcard $(RESDIR)/Braking*.vec)
# change suffix from .vec to .Rdata and add the bra prefix
BRAKING_DATA    = $(BRAKING:$(RESDIR)/%.vec=$(RESDIR)/bra.%.Rdata)
# match all .vec files for the Sinusoidal config
SINUSOIDAL      = $(wildcard $(RESDIR)/Sinusoidal*.vec)
# change suffix from .vec to .Rdata and add the sin prefix
SINUSOIDAL_DATA = $(SINUSOIDAL:$(RESDIR)/%.vec=$(RESDIR)/sin.%.Rdata)

# vector index files and Rdata files
VCI = $(VECTOR:%.vec=%.vci)
RDATA = $(VECTOR:%.vec=%.Rdata)

# all make targets
all: Braking.Rdata Sinusoidal.Rdata 

# to make Braking.Rdata we need to merge all files starting with bra.Braking
# before this, check that all BRAKING_DATA files have been processed
$(RESDIR)/Braking.Rdata: $(BRAKING_DATA)
	Rscript $(MERGESCRIPT) $(RESDIR)/ bra.Braking $(notdir $@) map-config default
Braking.Rdata: $(RESDIR)/Braking.Rdata

# to make Sinusoidal.Rdata we need to merge all files starting with sin.Sinusoidal
# before this, check that all SINUSOIDAL_DATA files have been processed
$(RESDIR)/Sinusoidal.Rdata: $(SINUSOIDAL_DATA)
	Rscript $(MERGESCRIPT) $(RESDIR)/ sin.Sinusoidal $(notdir $@) map-config default
Sinusoidal.Rdata: $(RESDIR)/Sinusoidal.Rdata

# to make all bra.*.Rdata files we need to run the generic parser
bra.%.Rdata: %.vec %.vci
	Rscript generic-parser.R $< map-config default bra

# to make all sin.*.Rdata files we need to run the generic parser
sin.%.Rdata: %.vec %.vci
	Rscript generic-parser.R $< map-config default sin

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
