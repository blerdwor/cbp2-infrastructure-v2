// predict.cc
// This file contains the main function.  The program accepts a single 
// parameter: the name of a trace file.  It drives the branch predictor
// simulation by reading the trace file and feeding the traces one at a time
// to the branch predictor.

#include <stdio.h>
#include <stdlib.h>
#include <string.h> // in case you want to use e.g. memset
#include <assert.h>
#include <math.h>
#include <iostream>
#include <fstream>

#include "branch.h"
#include "trace.h"
#include "predictor.h"
#include "my_predictor.h"

std::ofstream logfile("output.txt", std::ios::app);

int main (int argc, char *argv[]) {	

	// make sure there is one parameter
	if (argc != 2) {
		fprintf (stderr, "Usage: %s <filename>.gz\n", argv[0]);
		exit (1);
	}

	// open the trace file for reading

	init_trace (argv[1]);

	// initialize competitor's branch prediction code

	branch_predictor *p = new my_predictor ();

	// some statistics to keep, currently just for conditional branches

	long long int 
		tmiss = 0, 	// number of target mispredictions
		dmiss = 0, 	// number of direction mispredictions
		total_misses = 0, // number of combined misprediction
		total_branches = 0;

	long long int total_conditional = 0;
	long long int total_indirect = 0;

	// keep looping until end of file

	for (;;) {

		// get a trace

		trace *t = read_trace ();

		// NULL means end of file

		if (!t) break;

		// send this trace to the competitor's code for prediction

		branch_update *u = p->predict (t->bi);

		// collect statistics for a conditional branch trace

		total_branches++;

		// compare to gshare mispredictions for dmiss and tmiss

		if (t->bi.br_flags & BR_CONDITIONAL) {
			
			// logfile << (u->direction_prediction () == t->taken) << std::endl;

			// count a direction misprediction
			total_conditional++;

			dmiss += u->direction_prediction () != t->taken;

			// if (logfile.is_open()) {
			// 	logfile << t->bi.address << " " << t->taken << " " << u->direction_prediction () << "\n";
			// }
		}

		// collect statistics for an indirect branch trace

		if (t->bi.br_flags & BR_INDIRECT) {
			// logfile << (u->target_prediction () == t->target) << std::endl;
			// count a target misprediction
			total_indirect++;

			tmiss += u->target_prediction () != t->target;
		}

		// update competitor's state

		p->update (u, t->taken, t->target);
	}

	// logfile.close();

	// done reading traces

	end_trace ();

	total_misses = dmiss + tmiss;

	logfile << "dmiss: " << dmiss << " " << total_conditional << std::endl;
	logfile << "tmiss: " << tmiss << " " << total_indirect << std::endl;
	logfile << "total branches: " << total_branches << std::endl;
	logfile << "total miss: " << total_misses << std::endl << std::endl;

	// give final mispredictions per kilo-instruction and exit.
	// each trace represents exactly 100 million instructions.

	printf ("%0.3f MPKI\n", 1000.0 * (total_misses / 1e8));
	// printf ("%0.3f MPKI\n", 1000.0 * (tmiss / 1e8));
	// printf ("%0.3f MPKI\n", 1000.0 * (total_misses / 1e8));
	delete p;
	exit (0);
}
