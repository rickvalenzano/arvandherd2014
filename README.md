This is the source code for ArvandHerd, which won the multi-core track of the 2014 International Planning Competition. It includes some improvements over the version of the planner which also won the 2011 International Planning Competition. Below, you can find a list of the relevant papers describing this planner, and notes on running it.

# Relevant Publications

A description of the planner can be found in the following three publications.

Richard Valenzano, Hootan Nakhost, Martin Müller, Jonathan Schaeffer,
and Nathan Sturtevant. “ArvandHerd 2014.” The 2014 International Plan-
ning Competition: Description of Planners, Deterministic Track, pages 1–5, 2014.

Richard Valenzano, Hootan Nakhost, Martin Müller, Jonathan Schaeffer,
and Nathan Sturtevant. “ArvandHerd: Parallel Planning with a Portfolio.”
In the Proceedings of the 20th European Conference on Artificial Intelligence,
pages 786-791, 2012.

Richard Valenzano, Hootan Nakhost, Martin Müller, Jonathan Schaeffer,
and Nathan Sturtevant. “ArvandHerd: Parallel Planning with a Portfolio.”
In The 2011 International Planning Competition: Description of Participating
Planners, Deterministic Track, pages 113–116,2011.

# Planner Details

The provided code allows for a user to run the
versions of ArvandHerd that competed in the multi-core, single-core,
and agile tracks of IPC 2014. They only differ in the way they are
parameterized. r ArvandHerd2014. The parameters that differed between
the planners is determined by the configuration file. All three
configuration files have been included in the attached file. To change
which one is being used, simply change which one is being called on
line 15 of the "plan" file. For example, currently that line looks as
follows:

"$SEARCH" "-seed" "1983" "-iterative" "-conf_file" "sat_conf" "-g"
all.groups "-o" "$3" < output

"-conf_file" specifies that the next argument is a configuration file,
which in this case is the satisficing configuration file "sat_conf".
If you want to use the multi-core file, simply replace "sat_conf" with
"mc_conf". This means that the line should look as follows:

"$SEARCH" "-seed" "1983" "-iterative" "-conf_file" "mc_conf" "-g"
all.groups "-o" "$3" < output

The agile configuration file "agile_conf" can be used similarly.

For more information on how the parameters are used, first use the
build command to make the planner binary. This will create a binary
called "search" inside the "search" folder. Go into this folder, and
run ".search -help" to get information on the parameters. Of
particular note, "-seed" changes the seed for the random number
generator, and "-iterative" makes the planner continue to look for
solutions after a first solution has been found. If you remove
"-iterative", it will stop after a first solution is found. The
lengths of random walks, random walk biasing approach, and Aras
parameters are particularly important to planner performance.
