.. _mongodb:

===============
Logging Results
===============

One particularly useful feature in RoboSherlock is the logging of results and that of the raw data into a database, for later inspection. You will create a new pipeline (aggregate analysis engine) that stores raw data from the bag file in a mongodb, then modify the same AE, to read the data out of the database, process it, and store the results back in the database. 

Start by copying over the ``storage.xml`` AE descriptor from ``{..}/robosherlock/descriptors/analysis_engines/`` to your package (correct folder is descriptors/analysis_engines), and for example rename it to ``dbwriter.xml``. Aternatively you can also modify the ``my_demo.xml`` that you used in the previous tutorials, to have the same pipeline defined as in ``sorage.xml``.

.. note:: Copying it in the correct folder is important, since during runtime if the AE you want to run is not in your working directory, it will not be found. Having it in the correct directory, the run script searches for all projects that are dependent on RoboShelrock in your catkin workspace and looks in the ``descriptors/analyisis_engines`` folder to find the AE supplied as a parameter.

If you choose to copy over the ``storage.xml`` and renamed it don't forget to run ``catkin_make`` in order to regenerate the paths to the annotators. Once compilation finished you may start executing the pipeline (don't forget a roscore and running the test bagfile). To summ it up here are the commands you need to run::

  roscore
  rosrun robosherlock run dbwriter -visualizer
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

Even though the ImagePrepocessor component creates a point cloud, by default these are not stored in the database out of storage space considerations. Storing them can be enabled though by adding the keyword ``cloud`` to the ``enableViews`` parameter of the ``StorageWriter.xml`` located in ``{..}/robosherlock/descriptors/annotators/io``. However, changing the default parameters is not recommended, instead you can overwrite them in the AE definition.
Edit the AE that we just executed and add the following lines before the ``<flowconstrained>`` tag (remember to remove the empty ``</configurationParameters>`` and ``<configurationParameterSettings>`` tags before):

.. code-block:: xml

    <configurationParameters>
        <configurationParameter>
           <name>enableViews</name>
           <type>String</type>
           <multiValued>true</multiValued>
           <mandatory>false</mandatory>
           <overrides>
               <parameter>StorageWriter/enableViews</parameter>
           </overrides>    
        </configurationParameter>
    </configurationParameters>
    <configurationParameterSettings>
        <nameValuePair>
          <name>enableViews</name>
          <value>
              <array>
                <string>color_image_hd</string>
                <string>depth_image_hd</string>
                <string>camera_info</string>
                <string>camera_info_hd</string>
                <string>scene</string>
                <string>cloud</string>
              </array>
          </value>
        </nameValuePair>
    </configurationParameterSettings>

Now execute the pipeline again as described above and notice that there is a new collection called ``cloud`` that is stored in the database::

	mongo
	use Scenes
	show collections
	
It is not very convenient to always have to play a bag file in order to get data just for testing. Now that you have the raw data stored in the database, you can easily read it out from there, and execute pipelines on it. Modify your previous AE (my_demo.xml) to make it read from a database instead of listening to topics, and add a StorageWriter to the end of the pipeline it defines to store all results. 

Start with changing the interface the CollectionReader uses. This is done by overwriting the parameter ``camera_config_files`` from the ``CollectionReader.xml``. The variable is already overwritten in your ``my_demo.xml``, so simply change the following line (highlighted):

.. code-block:: xml
   :emphasize-lines: 5

    <nameValuePair>
        <name>camera_config_files</name>
        <value>
          <array>
            <string>config_mongodb_playback.ini</string>
          </array>
        </value>
    </nameValuePair>


.. note:: All configuration  files are located in the ``./config`` folder of the ``robosherlock`` package. By default the ``config_mongodb_playback.ini`` reads from the Scenes database, so we don't have to modify it. Currently it is not possible to have configuration files in other places than this folder. 

Run the modified pipeline, without playing a bagfile now::

  rosrun robosherlock run my_demo -visualizer
  
Notice that the execution will continue to loop and never stop. This is because the configuration file for playing back data from the mongo database is set to loop infinitely. You can stop execution by selecting one of the visualizer windows and hit escape, or from the terminal using ``Ctrl+C``. 

Finally you will add a StorageWriter component to ``my_demo.xml`` and write the restuls of the new pipeline into a new database called Scenes_annotated. 
Start by adding the ``StorageWriter`` component to the pipeline:

.. code-block:: xml
   :lineno-start: 133 
   :emphasize-lines: 12
   
   <fixedFlow>
   <node>CollectionReader</node>
   <node>ImagePreprocessor</node>
   <node>PointCloudFilter</node>
   <node>NormalEstimator</node>
   <node>PlaneAnnotator</node>
   <node>ImageSegmentationAnnotator</node>
   <node>PointCloudClusterExtractor</node>
   <node>ClusterMerger</node>
   <node>MyFirstAnnotator</node>
   <node>ResultAdvertiser</node>
   <node>StorageWriter</node>
   </fixedFlow>

Now owerwrite the parameter ``storagedb`` from the StorageWriter. Add the following to ``my_demo.xml``, in their respective places:

.. code-block:: xml

	<configurationParameter>
		<name>storagedb</name>
        <type>String</type>
        <multiValued>false</multiValued>
        <mandatory>false</mandatory>
        <overrides>
            <parameter>StorageWriter/storagedb</parameter>
        </overrides>    
    </configurationParameter>

    <nameValuePair>
        <name>storagedb</name>
        <value>
            <string>Scenes_annotated</string>
        </value>
    </nameValuePair>
    
Run the new pipeline and inspect the results in the mongodb. Optionally you can turn off looping in the configuration file, so execution halts once all frames have been processed. You could search for the annotation you had previously created, it will be stored in the scene collection in the annotations array of each identifiable.
