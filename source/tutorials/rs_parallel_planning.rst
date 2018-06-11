.. _rs_parallel_planning:

===================================
Using RS Parallel Pipeline Planning
===================================

Current status
---------------------

The project is going on track as required by the scope of RoboSherlock on this year GSOC. ::

Two things have been completed: ::
* Implement RSParallelPipelinePlanner that is able to mark execution orderings of annotators based on their required inputs and outputs.
* Examine UIMACPP code repository, gain a deep level of how a AnalysisEngine execution calls annotators' process.

All implementation so far does not require new dependencies. ::
The code base is on branch parallelism-dev. [Link](https://github.com/anindex/robosherlock/tree/parallelism-dev)

Underlying models
---------------------

This section will explain how RSParallelPipelinePlanner works. RSParallelPipelinePlanner consists of these features: ::
* Small interface as function to query annotator input output data from knowrob and store as std::map of annotator name and pair of input output list.
* Directed Graph data structure to store dependency info of annotators.
* Dependencies graph builder.
* Orderings marker based on graph traveling algorithm.
* Ordering refiner to treat standalone annotators to execute last.

At current state, RSParallelPipelinePlanner works closely with RSControledAnalysisEngine. When a query is posted, the planner is triggered to query input output requirements of new pipeline annotator list. It then build dependency graph and run ordering marker, the pipeline structure is finally extracted from the markers.
This model is foundation for next phase implementation of parallel annotators' execution.

Setting up and demo
---------------------

This assumes that user has installed knowrob, robosherlock_knowrob and robosherlock_msgs. All workspace are sourced and used python catkin build. ::

Compiling: ::

``
cd ~/<YOUR_WS>/src
git clone https://github.com/anindex/robosherlock
cd robosherlock && git checkout parallelism-dev
cd ../.. && catkin build
``

Setting up: ::

``
roscore&
roslaunch robosherlock rs.launch ae:=demo
roslaunch json_prolog json_prolog.launch initial_package:=robosherlock_knowrob
``

Demo (user can use any query json message): ::

``
rosservice call RoboSherlock/query "query: '{\"detect\":{\"color\":\"blue\", \"shape\":\"flat\"}}'"
``
User can see orderings info on robosherlock terminal. ::

Considerations
---------------------

Some issues need to be solved later (important!): ::
* Annotators need fail safe mechanisms for not causing crash for the pipelines.
* Some annotators are not satisfied input requirements, for example: ObjectIdentityResolution.
