# tool for indexing vec files
SCAVETOOL = scavetool
# scripts location
SCRIPTDIR = .
# results location
RESDIR = ../results
# script for merging
MERGESCRIPT = $(SCRIPTDIR)/merge.R

# match all .vec files for the Highway config
FAILURESSTATS        = $(wildcard $(RESDIR)/Highway*.vec)
# change suffix from .vec to .Rdata and add the failures prefix
FAILURESSTATS_DATA   = $(FAILURESSTATS:$(RESDIR)/%.vec=$(RESDIR)/failures.%.Rdata)
# match all .vec files for the Highway config
JOINSSTATS           = $(wildcard $(RESDIR)/Highway*.vec)
# change suffix from .vec to .Rdata and add the js prefix
JOINSSTATS_DATA      = $(JOINSSTATS:$(RESDIR)/%.vec=$(RESDIR)/js.%.Rdata)
# match all .vec files for the Highway config
CARSNUMBERSTATS      = $(wildcard $(RESDIR)/Highway*.vec)
# change suffix from .vec to .Rdata and add the cn prefix
CARSNUMBERSTATS_DATA = $(CARSNUMBERSTATS:$(RESDIR)/%.vec=$(RESDIR)/cn.%.Rdata)

# vector index files and Rdata files
VCI = $(VECTOR:%.vec=%.vci)
RDATA = $(VECTOR:%.vec=%.Rdata)

# all make targets
all: FailuresStats.Rdata JoinsStats.Rdata CarsNumberStats.Rdata 

# to make FailuresStats.Rdata we need to merge all files starting with failures.Highway
# before this, check that all FAILURESSTATS_DATA files have been processed
$(RESDIR)/FailuresStats.Rdata: $(FAILURESSTATS_DATA)
	Rscript $(MERGESCRIPT)
FailuresStats.Rdata: $(RESDIR)/FailuresStats.Rdata

# to make JoinsStats.Rdata we need to merge all files starting with js.Highway
# before this, check that all JOINSSTATS_DATA files have been processed
$(RESDIR)/JoinsStats.Rdata: $(JOINSSTATS_DATA)
	Rscript $(MERGESCRIPT) $(RESDIR)/ js.Highway $(notdir $@) map-config default
JoinsStats.Rdata: $(RESDIR)/JoinsStats.Rdata

# to make CarsNumberStats.Rdata we need to merge all files starting with cn.Highway
# before this, check that all CARSNUMBERSTATS_DATA files have been processed
$(RESDIR)/CarsNumberStats.Rdata: $(CARSNUMBERSTATS_DATA)
	Rscript $(MERGESCRIPT)
CarsNumberStats.Rdata: $(RESDIR)/CarsNumberStats.Rdata

# to make all failures.*.Rdata files we need to run the generic parser
failures.%.Rdata: %.vec %.vci
	Rscript generic-parser.R $< map-config failures failures

# to make all js.*.Rdata files we need to run the generic parser
js.%.Rdata: %.vec %.vci
	Rscript generic-parser.R $< map-config default js

# to make all cn.*.Rdata files we need to run the generic parser
cn.%.Rdata: %.vec %.vci
	Rscript generic-parser.R $< map-config carsNumber cn

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
