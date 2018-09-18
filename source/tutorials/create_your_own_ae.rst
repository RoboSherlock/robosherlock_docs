.. _create_your_own_ae:

========================
Write your own Annotator
========================

This tutorial assumes that the reader has created a robosherlock package, and already run the pipeline described in the :ref:`first tutorial<pipeline>`.

As described in the :ref:`overview<overview_rs>`, analysis engines can be either primitive or aggregate. A primitive analysis engine is called an annotator. In the following the creation of a new primitive analysis engine will be described, followed by creating an aggregate AE that uses is.

.. note:: it is not required that all primitive analysis engines annotate a scene, but for simplicity we call individual experts annotators. E.g. primitive AEs can generate object hypotheses, or have I/O tasks

Create your annotator
---------------------

Besides the implementation, it is mandatory in the UIM framework to have meta definitions of every component. A small script is available that makes creating new components a bit faster. Execute::
  
  rosrun robosherlock rs_new_annotator rs_tutorial MyFirstAnnotator

which will create a new annotator called *MyFirstAnnotator* in the previously created ROS-package *rs_tutorial*. It creates an yaml meta file in *descriptors/annotators* and a source file in *./src*. It also adds the necessary lines to your CMakeLists.txt::

  rs_add_library(rs_myFirstAnnotator src/MyFirstAnnotator.cpp)
  target_link_libraries(rs_myFirstAnnotator ${CATKIN_LIBRARIES})

Every component in RoboSherlock is a  C++ library, that gets loaded during runtime. The implementation consists of a cpp file and a yaml descriptor.

The yaml descriptor
------------------

Confgiruations (meta definitions) of annotators are defined for every annotator in ``yaml`` files located in the ``<package_name>/descriptors/annotators`` folder. The annotator thatwe just created has the following configuration file:

.. code-block:: yaml
    annotator:
        name: MyFirstAnnotator
        implementation: rs_myFirstAnnotator
    parameters:
        test_param: 0.01
    capabilities:
        inputs: {}
        outputs: {}

The most important part of this configuration file is the implementation name. This is the name of a dynamic library that is the implementation of the annotator. All other parts of the configuration are optional, but this one is mandatory. 

Tha param section defines configuration parameters that the annotator has. These can be of type ``string, float, int, boolean`` or arrays of. The last part can help define capabilities. This part is useful if we are using the pipeline planning and knowledge integration of the system, allowing users to set i/o constraints for annotators.
	  
   
The cpp implementation
----------------------

`MyFirstAnnotator.cpp` was generated in the ``src`` folder::
    
	#include <uima/api.hpp>

	#include <pcl/point_types.h>
	//RS
	#include <rs/types/all_types.h>
	#include <rs/scene_cas.h>
	#include <rs/utils/time.h>

	using namespace uima;

	class MyFirstAnnotator : public Annotator
	{
	private:
	  float test_param;

	public:

	  TyErrorId initialize(AnnotatorContext &ctx)
	  {
	    outInfo("initialize");
	    ctx.extractValue("test_param", test_param);
	    return UIMA_ERR_NONE;
	  }
	
	  TyErrorId destroy()
	  {
	    outInfo("destroy");
	    return UIMA_ERR_NONE;
	  }
	
	  TyErrorId process(CAS &tcas, ResultSpecification const &res_spec)
	  {
	    outInfo("process start");
	    rs::StopWatch clock;
	    rs::SceneCas cas(tcas);
	    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
	    outInfo("Test param =  " << test_param);
	    cas.get(VIEW_CLOUD,*cloud_ptr);
	
	    outInfo("Cloud size: " << cloud_ptr->points.size());
	    outInfo("took: " << clock.getTime() << " ms.");
	    return UIMA_ERR_NONE;
	  }
	};
	
	// This macro exports an entry point that is used to create the annotator.
	MAKE_AE(MyFirstAnnotator)

Implementation of an annotator extends the ``Annotator`` class of the uimacpp library. ``Annotator`` has several virtual methods defined out of which we are overriding the ``initialize``, ``destroy`` and ``process`` functions. Since annotators get compiled into runtime libraries they must end with the ``MAKE_AE(<AnnotName>)`` macro, that exports the entry point.

The three methods that we overwrite implement the functionalities of the annotator:

	- ``initialize`` : gets called in the constructor of the class. Has the same functionalities as a constructor. We can read in the parameters defined in the xml descriptor here (in the tutorial code this is *test_param*).
	- ``destroy`` :  It's like a destructor of a class, e.g. deallocate memory, if needed. 
	- ``process`` :  this is where all the processing code goes. In the tutorial we convert the cas to the SceneCas, get the point cloud that we stored in it and display it's size

.. note:: ``SceneCas`` is a wrapper for the uima::CAS class from uimacpp for conveniently setting and getting data. 


You can now compile it with catkin_make or catkin build (we recommend using ``catkin-tools``.


Add it to an AE and run
-----------------------

In the previous  :ref:`tutorial <create_your_rs_catkin_pkg>` we copied over the demo.yaml to our poroject and renamed it to ``my_demo.yaml``. Open it and add your new annotator to the pipeline by adding it to the fixed flow:
   
Run the pipeline as described in :doc:`pipeline`. Look at the output in your terminal. There should be an output with the value of the test parameter, and the number of points in the point cloud. 

.. note:: It is recommended to create you own launch file in the current package. Notice that you have to change the arguments of the ros node in the launch file in order to execute your new pipeline( from demo to my_demo)

.. warning:: The annotators execute in the order they are defined in the fixed flow. Since the demo annotator accesses point clouds it needs to be put after the ImagePreprocessor component, since this is the module that creates the point cloud from the depth and rgb images. 

The output in the terminal should look like this::

   MyFirstAnnotator.cpp(40)[process] process start
   MyFirstAnnotator.cpp(44)[process] Test param =  0.01
   MyFirstAnnotator.cpp(47)[process] Cloud size: 307200
   MyFirstAnnotator.cpp(48)[process] took: 2.37502 ms.

