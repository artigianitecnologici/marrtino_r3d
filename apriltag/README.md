Se usate una telecamera diversa da https://images-na.ssl-images-amazon.com/images/I/71OsRuXVNIL._AC_SL1500_.jpg fare la calibrazione della telecamera con camera_calibration (http://wiki.ros.org/camera_calibration#Tutorials), spostare il file ottenuto $(nome_camera).yaml da ~/.ros/camera_info e metterlo dentro il package usb_cam (oppure cambiare il parametro camera_info_url dal launch file). Inoltre cambiare i vari parametri tipo camera_name nel launch file. 

se non lo avete già fatto mettete tags.yaml in apriltag_ros/config/ (è il file che identifica quali tag riconoscere, io ci ho messo i primi quattro del gruppo 36h11)

Il topic /tag_detections pubblica la distanza tra il tag riconosciuto e la telecamera. c'è anche /tag_detections_image che pubblica in streaming la telecamera con i tag evidenziati.

marrtino_frames.launch serve principalmente per visualizzare su rviz il modello di marrtino aggiungendo il display RobotModel e tf. Controllate che la telecamera sia posizionata nel posto giusto (è il blocco celeste).

per visualizzare il riconoscimento dei tag su rviz:
  lanciare i due file di launch
  aprire rviz e settare come fixed_frame base_link
  aggiungere i display Camera (con topic tag_detections_image), RobotModel, e tf
