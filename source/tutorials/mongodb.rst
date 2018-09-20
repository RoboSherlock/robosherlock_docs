.. _mongodb:

===============
Logging Results
===============

One particularly useful feature in RoboSherlock is the logging of results and that of the raw data into a database, for later inspection. You will create a new pipeline (aggregate analysis engine) that 
stores raw data from the bag file in a mongodb, then modify the same AE, to read the data out of the database, process it, and store the results back in the database. 

We will run an AE for storing the scenes in a mongoDB and modify the ``my_demo.yaml`` from the previous tutorials to read data from the database instead of listening ROS topics. To store the images in a database run the following::

  rosrun robosherlock run _ae:=storage
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

Even though the ImagePrepocessor component creates a point cloud, by default these are not stored in the database out of storage space considerations. Storing them can be enabled though by adding the keyword ``cloud`` to the ``enableViews`` parameter of the ``StorageWriter.yaml`` located in ``{..}/robosherlock/descriptors/annotators/io``. However, changing the default parameters is not recommended, instead you can overwrite them in the AE definition.


It is not very convenient to always have to play a bag file in order to get data just for testing. Now that you have the raw data stored in the database, you can easily read it out from there, and execute pipelines on it. Modify your previous AE (my_demo.xml) to make it read from a database instead of listening to topics, and add a StorageWriter to the end of the pipeline it defines to store all results. 

Readin raw data in RoboSherlock is handled by the CollectionReader. The config file for CollectionReader (located in ``descriptors/annoators/io``) looks like this

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

The collection reader takes a single paramteter (highlighted above), which ironically is a list of config files (this interface is due to change in future releaseses). This is becaus the CollectionReader can handle multiple input sources and they take different params. For example to read data from a camer we use the ROS interfaces (image and cam infor topics + TF locations), on the other hand reading from a database requires the name of the database. These config files can be defined in the ``config`` folder of any ROS package depening on RoboSherlock. Let's create one that reads from the previously stored database, by copying over an existing one from the core package::

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
    semanticMap=semantic_map.yaml
    
Now modify ``my_demo.xml``. First change the interface the CollectionReader uses. To do this by overwrite the parameter ``camera_config_files`` from the ``CollectionReader.yaml``. The variable is already overwritten in your ``my_demo.xml``, so simply change the value of it. Add a StorageWriter to the end of the fixed flow so you can save the restults of the pipeline. This time let's store the data in a DB called ``ScenesAnnotated``. To do this the ``storagedb`` param of ``StorageWriter`` needs to be overwritte. ``my_demo`` should look like this now (changes made to it are highlighted:

.. code-block:: yaml
   :emphasize-lines: 15, 17-19
      
      ae:
	name: my_demo
      fixedflow:
	- CollectionReader
	- ImagePreprocessor
	- PointCloudFilter
	- NormalEstimator
	- PlaneAnnotator
	- ImageSegmentationAnnotator
	- PointCloudClusterExtractor
	- ClusterMerger
	- Cluster3DGeometryAnnotator
	- MyFirstAnnotator
	- DrawResultImage
	- StorageWriter
      CollectionReader:
	camera_config_files: ['config_mongodb_example.ini']
      StorageWriter:
	storagedb: 'ScenesAnnotated'

Run the modified pipeline, no need to play the bagfile anymore::

  rosrun robosherlock run _ae:=my_demo 
  
Notice that the execution will continue to loop and never stop. This is because the configuration file for playing back data from the mongo database is set to loop infinitely. You can stop execution by selecting one of the visualizer windows and hit escape, or from the terminal using ``Ctrl+C``. 

    
Inspect the results in the mongodb. Optionally you can turn off looping in the configuration file, so execution halts once all frames have been processed::

    mongo
    show dbs
    use ScenesAnnotated
    db.scenes.find() 
    

  

