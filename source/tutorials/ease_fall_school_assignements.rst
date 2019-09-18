.. _ease_fall_school_assignements:

#######################################
RoboSherlock Tutorial: EASE Fall School
#######################################


*******************************
I. Introduction to RoboSherlock
*******************************

Befor you start update the docker image and make sure you have a container running::

    docker pull robosherlock/rs_interactive

If you did not have the latest image and you already have a container running stop it and restart it.

Cherk for a running container::
  
    docker ps -a
    
If you need to restart it execute the following::

    docker stop rs_demo
    docker rm rs_demo
    docker run -d -p 3000:3000 -p 8080:8080 -p 5555:5555 -p 9090:9090 -p 8081:8081 -v ${HOME}/sandbox:/home/rs/sandbox --name rs_demo robosherlock/rs_interactive

Open a terminal in your web browser::

    localhost:3000

RoboSherlock is a ROS package, and uses ROS to interface with other components of a robotic system. Before you begin let's set up a new ROS workspace. 


I.1. ROS workspace setup
========================

In your users home create a folder for the new workspace and initialize it as a catkin worksapce::
    
    mkdir -p demo_ws/src
    cd demo_ws
    catkin init 
    catkin config --extend /home/rs/rs_ws/devel
   
The last command here ensures that you are chaining the workspaces together correctly. Build the empty workspace::
    
    catkin build

and source it::

   source /home/rs/demo_ws/devel/setup.bash

Make sure to add it to your ``bashrc`` so that terminals that you open in the future will know about it::

    echo 'source /home/rs/demo_ws/devel/setup.bash' >> ~/.bashrc



I.2. Create your own catkin-robosherlock package
================================================

Let's create a new package called ``rs_tutorial``. Make sure you are in the source folder of the newly created catkin workspace and run::

    rosrun robosherlock rs_create_package rs_tutorial

The script will create a new catkin package that has the structure needed for Robosherlock.::

   'package_name'
    |-descriptors         
       |-analysis_engines -> yaml definitions of aggregate AEs
       |-annotators       -> yaml definitions of primitive AEs
       |-typesystem       -> yaml deginitions of the typesystem
          |-all_types.xml -> typesystem definition
    |-include
       |-package_name     -> include folder
          |-types         -> folder for the auto-generated type implementations
    |-src                 -> code base
    |-package.xml         -> catkin package xml   
    |-CMakeLists.txt      -> CMake file


It edits the *CMakeLists.txt*, sets the CMake variables needed for code generation. finding annotations automatically::

  ################################################################################
  ## Constants for project                                                      ##
  ################################################################################
  set(NAMESPACE package_name)
  set(TYPESYSTEM_CPP_PATH ${PROJECT_SOURCE_DIR}/include/package_name/types)
  set(TYPESYSTEM_XML_PATH ${PROJECT_SOURCE_DIR}/descriptors/typesystem)
  set(ANNOTATOR_PATH      ${PROJECT_SOURCE_DIR}/descriptors/annotators)
  set(ENGINE_PATH         ${PROJECT_SOURCE_DIR}/descriptors/analysis_engines)


  ################################################################################
  ## Update analysis engines, typesystem and include all relevant files         ##
  ################################################################################
  
  ## generate classes from the typesystem xml files
  generate_type_system(robosherlock)
  #find all relevant files
  find_additional_files()

The first part sets the five CMake variables that are in turned used by the scripts that are called in the second part. 
	
	* generate_type_system: checks if we have newly defined types in the xml descriptions and generates the C++ container classes for them
	
You can now add your custom annotators and pipeline analysis engines that can use any component defined in the RoboSherlock core package. If you want ``rs_tutorial`` to depend on other robosherlock packages add them to the ``package.xml`` and ``CMakeLists.txt``. We will do this in a later step. 


I.3. Running a pipeline in RoboSherlock
=======================================

.. This tutoial assumes that you have followed the tutorial on :ref:`creating a new robosherlock package <create_your_rs_catkin_pkg>`.

Let's first look at the main components of the framework, and how to use them. The docker image comes with a simple example data. Download the provided. You will find the bagfile in ``~/data/`` folder.  The bagfile was recorded using a PR2 robot and contains a short stream of kinect data, namely the topics (check with ``rosbag info``): ::
  
    /kinect_head/rgb/image_color/compressed
    /kinect_head/depth_registered/comressedDepth
    /kinect_head/rgb/camera_info
    /tf

TF is needed to get the transformation between camera frame and map or robot base. This feature can be turned off in the camera configuration files.

Perception pipelines in RoboSherlock are defined as aggregate analysis engines in the ``descriptor/analysis_engines`` folder of any robosherlock package. The core robosherlock package offers an executable called ``runAAE`` that we can use to run any of these pipelines. 
To see how this work we have prepared an example launch file in the ``rs_ease_fs`` package caleld ``ease_fs_demo.yaml``. Try and run the aggregate analysis engine from robosherlock. Start a roscore and in a second terminal launch the AAE execution:: 
    
    roscore
    roslaunch rs_ease_fs rs.launch
    
This will initialize active components of RoboSherlock and will wait for data to be published on the camera topics. The executable takes several rosparams as input, one of them being the name of the aggregate analysis engine we want to execute. To see more options run with ``--help`` option. For now just use the default parameters.  To actually process some images we will need to play the bagfile. Since it is only five seconds long loop it::    
    
    rosbag play ${HOME}/data/example.bag --loop
   
You can look at the results of the individual annotators using the browsed and visualizer page. Go to ``localhost:8081`` where you should see the following: 

    .. image:: ../imgs/ease_fs/localhost_8081.png
      :align: center
      :height: 20pc
    ..    :width: 100pc

Choose output image and the segmentation results should appear:

    .. image:: ../imgs/ease_fs/rs_output_image.png
      :align: center
      :height: 20pc
    ..    :width: 100pc

In order to view the results of the individual annotators, that make up the pipeline, we have created two commands  for swiching: ``rs_next`` and ``rs_prev``. Execute these commands in a terminal window and see the results in the visualization tab:

    .. image:: ../imgs/ease_fs/rs_next.png
      :align: center
      :height: 20pc
    ..    :width: 100pc


The demo is a very simple example of how perception pipelines are defined and one way of running them in RoboSherlock. The definition of the pipeline is located in 
*rs_ease_fs/descriptors/analysis_engines/demo.yaml*. Contents of it are the following:

.. code-block:: yaml
   
    ae: # -> various meta data	
        name: ease_fs_demo
    fixedflow: # -> the fixedflow a.k.a the perception algorithms, i/o components etc.
        - CollectionReader
        - ImagePreprocessor
        - PointCloudFilter
        - NormalEstimator
        - PlaneAnnotator
        - PointCloudClusterExtractor
        - ClusterMerger
    CollectionReader: # parameter overrides for annotators
        camera_config_files: ['config_kinect_robot_ease_fs.ini']

        
Modify this pipeline. For this, make a copy of it in ``rs_tutorial/descriptors/analysis_engines/``, and call it ``my_demo.yaml``::

    cp ~/rs_ws/src/rs_ease_fs/descriptors/analysis_engines/ease_fs_demo.yaml  ~/demo_ws/rs_tutorial/descriptors/analysis_engines/my_demo.yaml


Now add a a new annotator to the pipeline, called *Cluster3DGeometryAnnotator*. Simply add a new entry to the list under the **fixedflow** tag.  Since the launch file you are starting simply executes whatever it finds under the ``fixedflow``, the order of algorithms is important. Add *Cluster3DGeometryAnnotator* before *PlaneAnnotator* and run the pipeline. You can specify the newly created yaml using a parameter for the launch file::

     roslaunch rs_ease_fs rs.launch ae:=my_demo

.. warning:: The YAML data format is sensitive to small mistakes. Indent using spaces and leave spaces after dases or colons.
      
If you check the terminal output or the visualization of this annotator you should see no results. This is because no hypotheses have been generated in the pipeline yet. Now add it after the *ClusterMerger* and relaunch RoboSherlock. You will now have the estimated 3D bounding box with a pose estimate for each cluster as well as a heuristic-based symbolic label for the size of the object. Check the visualizer for results. 

Try adding some more annotators: *PrimitiveShapeAnnotator*, *ClusterColorHistogramCalculator* and *DrawResultImage* and visualizing their reults. The *ClusterColorHistogramCalculator* exposes a parameter that we need to overwrite in *my_demo.yaml*.  Simply add these two lines to the end of the file:

.. code-block:: yaml
   
    ClusterColorHistogramCalculator: # parameter overwriting 
        semantic_labels: true


I.4. Write your own Annotator
=============================

Analysis engines (AEs) can be either primitive or aggregate. A primitive analysis engine (PAE) is also referred to as an annotator. In the following the creation of a new primitive analysis engine will be described, followed by creating an aggregate AE that uses is.

.. note:: It is not required that all primitive analysis engines annotate a scene, but for simplicity we call individual experts, that wrap perception algorithms, annotators. For example primitive AEs can generate object hypotheses, or have I/O tasks.

Create your annotator
---------------------

Annotators in RoboSherlock are written in C++. Besides the implementation, it is mandatory to have meta definitions of every component. A small script is available that makes creating new components faster. To create a new annotator, called ``MyFirstAnnotator`` in the package ``rs_tutorial``, run the following::
  
  rosrun robosherlock rs_new_annotator rs_tutorial MyFirstAnnotator

which will create a new annotator called *MyFirstAnnotator* in the previously created ROS-package *rs_tutorial*. It creates an yaml meta file in *descriptors/annotators* and a source file in *./src*. It also adds the necessary lines to your CMakeLists.txt::

  rs_add_library(rs_myFirstAnnotator src/MyFirstAnnotator.cpp)
  target_link_libraries(rs_myFirstAnnotator ${CATKIN_LIBRARIES})

Every component in RoboSherlock is a  C++ library, that gets loaded during runtime. The implementation consists of a cpp file and a yaml descriptor.

The yaml descriptor
-------------------

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

`MyFirstAnnotator.cpp` was generated in the ``src`` folder, replace its contents with the following source code::
    
      #include <uima/api.hpp>
      #include <pcl/point_types.h>
      #include <rs/types/all_types.h>
      //RS
      #include <rs/scene_cas.h>
      #include <rs/utils/time.h>
      #include <rs/DrawingAnnotator.h>
      using namespace uima;
      class MyFirstAnnotator : public DrawingAnnotator
      {
      private:
	float test_param;
	cv::Mat disp_;
      public:
	MyFirstAnnotator(): DrawingAnnotator(__func__)
	{
	}
	
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
	TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
	{
	  outInfo("process start");
	  rs::StopWatch clock;
	  rs::SceneCas cas(tcas);
	  
	  cas.get(VIEW_COLOR_IMAGE, disp_);   
	  rs::Scene scene = cas.getScene();
	  std::vector<rs::ObjectHypothesis> hyps;
	  scene.identifiables.filter(hyps);
	  outInfo("Found "<<hyps.size()<<" object hypotheses");    
	  for (auto h:hyps)
	  {
	    std::vector<rs::Shape> shapes;
	    std::vector<rs::SemanticColor> colors;
	    h.annotations.filter(shapes);
	    h.annotations.filter(colors);
	  
	    bool shapeMatch=false, colorMatch=false;
	    for (auto shape:shapes)
		if(shape.shape() == "flat")
		      shapeMatch = true;
	    for(auto color:colors)
		if(color.color() == "black")
		      colorMatch = true;

	    if(shapeMatch && colorMatch)
	    {
	      outInfo("We have found a flat black object");
	      cv::Rect rect;
	      rs::conversion::from(h.rois().roi(), rect);
	      cv::rectangle(disp_,rect,cv::Scalar(0,0,255));
	      rs::Detection detection = rs::create<rs::Detection>(tcas);
	      detection.source.set("MyFirstAnnotator");
	      detection.name.set("SeverinPancakeMaker");
	      detection.confidence.set(1.0);
	      h.annotations.append(detection);
	    }
	  } 
	  return UIMA_ERR_NONE;
	}
	void drawImageWithLock(cv::Mat &disp)
	{
	disp = disp_.clone();
	}
      };

      // This macro exports an entry point that is used to create the annotator.
      MAKE_AE(MyFirstAnnotator)


Let's break the code down.
Implementation of an annotator extends the ``Annotator`` class of the uimacpp library. ``Annotator`` has several virtual methods defined out of which we are overriding the ``initialize``, ``destroy`` and ``process`` functions. Since annotators get compiled into runtime libraries they must end with the ``MAKE_AE(<AnnotName>)`` macro, that exports the entry point.

The three methods that we overwrite implement the functionalities of the annotator:

	- ``initialize`` : gets called in the constructor of the class. Has the same functionalities as a constructor. We can read in the parameters defined in the xml descriptor here (in the tutorial code this is *test_param*).
	- ``destroy`` :  It's like a destructor of a class, e.g. deallocate memory, if needed. 
	- ``process`` :  this is where all the processing code goes. In the tutorial we convert the cas to the SceneCas, get the point cloud that we stored in it and display it's size

.. note:: ``SceneCas`` is a wrapper for the uima::CAS class from uimacpp for conveniently setting and getting data. 


You can now compile it by calling *catkin build*. 


Add it to an AE and run
-----------------------

Open ``my_demo.yaml`` and add your new annotator to the pipeline by adding it to the fixed flow. Run the pipeline as previously described Look at the output images. Switch until you see the result from ``MyFirsAnnotator`` (use the we visualization on port 8081).

.. warning:: Remember, the annotators execute in the order they are defined in the fixed flow. Since the demo annotator accesses hypotheses, shapes and colors it needs to be put after the components returning these.

In your output image you should see the pancake maker being detected, highlighted by a rectangle around it in the image.


5. Logging Results
==================

One particularly useful feature in RoboSherlock is the logging of results and that of the raw data into a database, for later inspection. You will create a new pipeline (aggregate analysis engine) that 
stores raw data from the bag file in a mongodb, then modify the same AE, to read the data out of the database, process it, and store the results back in the database. 

We will run an AE for storing the scenes in a mongoDB and modify the ``my_demo.yaml`` from the previous tutorials to read data from the database instead of listening ROS topics. To store the images in a database run the following::

  roslaunch rs_ease_fs rs.launch ae:=ease_fs_storage
  rosbag play test.bag

When the bagfile finishes playing stop the RoboSherlock instance and inspect the results in the database. The easiest way to do this is using a common tool like `RoboMongo <http://www.robomongo.org>`_ . Alternatively you can use the terminal tool that comes with mongodb. Start the mongo shell::

	mongo

Specify the database you want to use (the default database in RoboSherlock is Scenes)::
	
	use Scenes
  
Print the name of the collections that were created:: 

	show collections
	
There should be seven collection in the Scenes database::

	camera_info
	camera_info_hd
	cas
	color_image_hd
	depth_image_hd
	scene
	system.indexes

The main collection is the cas, where the index of each document is the timestamp of the frame that got processed. Raw data (color and depth image) as well as the processed scenes (scene) are referenced from here using their respective objectID-s. We can view the content and the number of documents in it by running::

	db.cas.find()
	db.cas.find().count()
	
Since the pipeline you run only contained a CollectionReader, ImagePreprocessor and the StorageWriter, your ``scene`` collection is going to be empty, and the database essentially only contains the raw images, and the camera info.

Even though the ImagePrepocessor component creates a point cloud, by default these are not stored in the database out of storage space considerations. Storing them can be enabled though by adding the keyword ``cloud`` to the ``enableViews`` parameter of the ``StorageWriter`` in ``storage_ease_fs.yaml`` located in ``{..}/rs_ease_fs/descriptors/ananalysis_engines``.

It is not very convenient to always have to play a bag file in order to get data just for testing. Now that you have the raw data stored in the database, you can easily read it out from there, and execute pipelines on it. Modify your previous AAE, ``my_demo.xml``, to make it read from a database instead of listening to topics, and add a StorageWriter to the end of the pipeline it defines to store all results. Let's see how this can be done.

Modiying an AAE to read from a DB
---------------------------------

Reading raw data in RoboSherlock is handled by the CollectionReader. The config file for CollectionReader (located in ``robosherlock/descriptors/annoators/io``) looks like this (you don't have to modify this file):

.. code-block:: yaml
  :emphasize-lines: 6
  
  annotator:
    name: CollectionReader
    implementation: rs_CollectionReader
    description: 'Uses Camera Bridges, as available, to fill the cas with sensor data.'
  parameters:
    camera_config_files: ['config_kinect_robot.ini']
  capabilities:
    outputs: ['rs.cv.Mat']

The collection reader takes a single paramteter (highlighted above), which is a list of config files (this interface is due to change in future releaseses). This is becaus the CollectionReader can handle multiple input sources and they take different params. For example to read data from a camer we use the ROS interfaces (image and cam infor topics + TF locations), on the other hand reading from a database requires the name of the database. These config files can be defined in the ``config`` folder of any ROS package that depends on RoboSherlock. Create an input configuration file that reads from the previously stored database, by copying over an existing one from the core RoboSherlock implementation::

    roscd rs_tutorial
    mkdir config
    cp $(rospack find robosherlock)/config/config_mongodb_playback.ini ./config/config_mongodb_example.ini

The content of the config file is the following::

    [camera]
    interface=MongoDB ->specifies the interface so COllectioReader knows which bridge to instantiate
    [mongodb]
    host=localhost ->IP of machine hosting the db
    db=Scenes -> database name
    continual=false ->if reached the last entry wait for new ones
    loop=true -> if reached the last entry start from beginning
    playbackSpeed=0.0 ->try to control the rate at which images are read in
    [tf]
    semanticMap=semantic_map_iai_kitchen.yaml
    
Now modify ``my_demo.xml``. First change the interface the CollectionReader uses. To do this change the value of the already overwriten the parameter of the ``CollectionReader``, ``camera_config_files``, from ``config_kinect_robot_ease_fs.ini`` to *config_mongodb_example.ini*. Add a StorageWriter to the end of the fixed flow so you can save the restults of the pipeline. This time let's store the data in a DB called ``ScenesAnnotated``. To do this the ``storagedb`` param of ``StorageWriter`` needs to be overwritte. Another paramter telling the system that we have a single camera needs to also be set (this is a hack, more on this later). ``my_demo`` should look like this now (changes made to it are highlighted):

.. code-block:: yaml
   :emphasize-lines: 16, 18, 21-23
      
      ae:
	name: ease_fs_demo
      fixedflow:
	- CollectionReader
	- ImagePreprocessor
	- RegionFilter
	- NormalEstimator
	- PlaneAnnotator
	- PointCloudClusterExtractor
	- ClusterMerger
	- Cluster3DGeometryAnnotator
	- PrimitiveShapeAnnotator
	- ClusterColorHistogramCalculator
	- MyFirstAnnotator
	- DrawResultImage
	- StorageWriter
      CollectionReader:
	camera_config_files: ['config_mongodb_example.ini']
      ClusterColorHistogramCalculator:
        semantic_label: true
      StorageWriter:
	storagedb: 'ScenesAnnotated'
	multi_cam: false

Run the modified pipeline, no need to play the bagfile anymore::

  roslaunch rs_ease_fs rs.launch ae:=my_demo 
  
Notice that the execution will continue to loop and never stop. This is because the configuration file for playing back data from the mongo database is set to loop infinitely. You can stop execution any time from the terminal by presing ``Ctrl+C`. 

    
Inspect the results in the mongodb. Optionally you can turn off looping in the configuration file, so execution halts once all frames have been processed::

    mongo
    show dbs
    use ScenesAnnotated
    db.scenes.count() 
    
Congrats, you have just logged **everything** that RoboSherlock has done. Next you will see how this data can be used to improve the perception of 

**************************************
II.Adapting capabilities based on logs
**************************************

Now that you have seen how to run a pipeline, and how to modify it, let's see how you can use the logged data to create a better detector. Start with inspecting the results and extracting data from the logs with the purpose of retraining detection components.

RoboSherlock offers a web-frontend for interaction. In a new terminal run::

  roslaunch rs_ease_fs rs_web.launch
  
In a browser navigate to ``localhost:5555``. For now disregard the canvas and predefined queries on the ``Live Query`` tab. Choose Objects Store tab and in the dropdown box select ScenesAnnotated as the active database, and click on Query:

.. image:: ../imgs/rs_web_selecting_database.png
   :align: center
   :width: 30pc
..  :height: 30pc

Use the web interface to inspect the results. You can view the individual scenes, or filter the hypotheses based on some predefined criteria. Go to the objects tab, query it. There should be no results. This is because objects are the results of entity resolution. RoboSherlock implements an heuristic based object identity resolution component. Add it to ``my_demo.yaml``, just before the ``StorageWriter`` and rerun the pipeline. If you have set the mongo playback's loop to false RS should terminate on its own, if not stop it after you think enough frames have been processed. 

Now go back to the web interface (you might have to refresh the web page) and look at the objects tab. There should be five objects. Click on se the export button to export the images. You will use these images as your new source data for training a simple k-NN. For this do the following.

Group the images into folders, based on the object they depict. You can do this on the host machine (use the ``sandbox`` folder so your docker will see the changes). The name of the folders is going to be the object instance. It is important that these objects have the following names::

  * VollMilch
  * SeverinPancakeMaker
  * KelloggsCornFlakes
  * MondaminPancakeMix
  * ComdoCappuccinoClassico

Next up, extract features for the objects. Create a split file in the sandbox folder. This is needed for the script extracting the features, call it ``my_split.yaml``::
    
    %YAML:1.0
    classes:
      - VollMilch (these names need to identical with the folder names)
      - SeverinPancakeMaker
      - KelloggsCornFlakes
      - MondaminPancakeMix
      - ComdoCappuccinoClassico

Now you can use the ``featureExtractor`` from ``rs_addons`` to extract CNN feature descriptors for each of our objects. In docker run::
  
    rosrun rs_addons featureExtractor -s /home/rs/sandbox/split.yaml -i /home/rs/sandbox/Objects/ -o /home/rs/sandbox
    
This will generate two files in the sandbox folder: ``BVLC_REF_ClassLabel_my_split.txt`` containin a class nr to class lable mapping and ``BVLC_REF_data_my_split.yaml`` containing the features extracted. 
Let's now set the ``KnnAnnotator`` to use these features to classify. In ``my_demo.yaml`` we will overwrite some of the parameters of KnnAnnotator:

.. code-block:: yaml
   :emphasize-lines: 15, 17-19
      
      ...
      CollectionReader:
	camera_config_files: ['config_mongodb_example.ini']
      StorageWriter:
	storagedb: 'ScenesAnnotated'
      KnnAnnotator:
        feature_descriptor_type: BVLC_REF
        class_label_mapping: /home/rs/sandbox/BVLC_REF_ClassLabel_split.txt
        trainin_data: /home/rs/sandbox/BVLC_REF_data_split.yaml

Now run ``my_demo`` again and inspect the results in the db using the web interface. You should see that the objects are correctly classified. Congrats, you just adapted the recognition capabilities of a robot based on it's episodic memories...sort of. Though the example is very simple the same techniques can be used to performs more complex tasks.  In the following we will take a look at how all of this can be used to answer queries that a robotic agent might ask. 

***************************************** 
Knowledge integration and query answering
*****************************************

The task of RoboSherlock is to complete the objects designators sent by the high-level executive; For this it offers a query interface. Queries get interpreted, a pipline is planned and executed.

If the web app from the preovious section is still running stop it. Launch knowrob and the web server together::

    roslaunch rs_run_configs json_prolog_and_rosbridge.launch 
    
This launches the json prolog interface to knowrob and all web frontend for interacting with RoboSherlock; You are going to be using the RS live tab of the web interface for this part of the tutorial. For the interactive visualization and query interface to work we need to start RoboSherlock using roslaunch. Copy one over from ``rs_run_configs`` and edit it so it launches ``my_demo.yaml``::

    roscd rs_tutorial
    mkdir launch
    cp $(rospack find rs_run_configs)/launch/wed_demo.launch./launch


Before launching robosherlock go to the ``Live query`` tab of the web frontend and start executing the predefined queries, up until the detection queries section.  

.. image:: ../imgs/tutorials/rs_live.png
   :align: center
   :width: 30pc
..    :height: 30pc
..    :width: 30pc


Once you retracted all assertions you can now launch robosherlock by running::

    roslaunch rs_tutorial web_demo.launch

Notice how on the last lines of RoboSherlocks outputs it is going to complain about MyFirstAnnotator not existing in the knowledge base. Don't worry about this for now. You will fix it later on. 

Got to the web interace and execute the detection queries. In the middle bottom pane you can view the partial results of the annotators. Try combining the detection queries, change values etc.  

.. image:: ../imgs/tutorials/rs_live_annotator_results.png
   :align: center
   :width: 30pc
..    :height: 30pc


Let's first fix the annotator feature extraction. Add an input value constraint to ``KnnAnnotator``:

.. note:: In the current implementation input restrictions and output domains need to be set in the individual yamls of annotators, and cannot be overwritten from ``my_demo.yaml``.  An extension is planned for allowing to do this from the AAE yamls. 

.. code-block:: yaml
  
  capabilities:
    inputs: 
        - rs.scene.MergedCluster
        - rs.annotation.Features: ['BVLC_REF']

For KnnAnnotator to be included in the pipeline planning process we now need a component that produces an annotation of type ``rs.annotation.Features`` of type ``BVLC_REF``. Modify ``CaffeAnnotator.yaml`` to output rs.annotation.Features of type BVLC_REF. 

.. code-block:: yaml

    capabilties:
        outputs:
         - rs.annotation.Fetures: ['BVLC_REF']
         
Restart RoboSherlock and execute a new detection query. Notice that not all annotators from the ``fixedflow`` are part of the planned pipeline.

 
Let's extend the tool-chain and add your package's annotations to the tool-chain. Create a second annotator called ``MySecondAnnotator``. Let's edit both of our annotators input and output requirements. 

As input requirements of ``MyFirstAnnotator`` let's add a shape annotation (``rs.annotation.Shape``) as cylinders, and as output a detection result (``rs.annotation.Detection``) with output domain Cups.

For ``MySecondAnnotator`` let's consider an algorithm that finds handles on cups. As input it will take classification annotations ``rs.annotation.Classification`` with input value constraint set to a specific cup (``CupEcoOrange``) and produces a detection of type Handle. Once modeled we need to update the ontoloy::

    roscd robosherlock_knowrob/owl
    rosrun robosherlock_knowrob generateOwlFromXMl.py

Restart json_prolog and RoboSherlock (in this order). Plan a pipeline for detection Cups. Now plan one for detecting Hanldes.
..TBC

    
