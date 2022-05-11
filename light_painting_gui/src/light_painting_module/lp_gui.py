#!/usr/bin/env python
#
# Software License Agreement (Apache 2.0 License)
# Copyright (c) 2022, The Ohio State University
# The Artificially Intelligent Manufacturing Systems Lab (AIMS)
#
# Author: A.C. Buynak
#
# Description:
# Plugin for ROS RQT to support Light Painting Project 
# 

import os
import rospy
import rospkg

import roslaunch

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtGui import QImage, QPixmap
from python_qt_binding.QtCore import Qt

import cv2

class LightPaintingPlugin(Plugin):

    def __init__(self, context):
        super(LightPaintingPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('LightPaintingPlugin')

        # Create QWidget
        self._widget = QWidget()

        # Filepath to UI file
        ui_file = os.path.join(rospkg.RosPack().get_path('light_painting_gui'), 'resource', 'LightPaintingPlugin.ui')

        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)

        # Give QObjects reasonable names
        self._widget.setObjectName('LightPaintingPluginUi')

        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget)


        #### Widget Specific Setup ####
        
        # Connect Buttons
        self._widget.t1_initialize.clicked.connect(self.on_t1_initialize_clicked)
        self._widget.t1_execute.clicked.connect(self.on_start_program_clicked)
        self._widget.t1_send_stop.clicked.connect(self.on_stop_program_clicked)
        self._widget.refresh_loaded_images.clicked.connect(self.refresh_image_options)

        # Connect Events
        self._widget.imagePicker.currentIndexChanged.connect(self._set_selected_image)

        # Initial Menu Population
        self.refresh_image_options()




    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog



    #############################################
    ### Light Painting GUI Specific Callbacks ###
    #############################################

    def refresh_image_options(self):

        # Directories
        dir_data = os.path.join(rospkg.RosPack().get_path('light_painting'), 'data')
        self.dir = [os.path.join(dp, f) for dp, dn, fn in os.walk(os.path.expanduser(dir_data)) for f in fn]

        # Clear Old Contents from ComboBox
        self._widget.imagePicker.clear()

        # Update ComboBox Widget
        for file in [f for f in self.dir if f.endswith('.tif') ]:
            filename = os.path.basename(file)
            self._widget.imagePicker.addItem(filename)


    def _set_selected_image(self, index):

        if index>=0:
            self.selected_image_path = self.dir[index]

            rospy.set_param("selected_image_path", self.selected_image_path)

            # Display parent folder name        
            foldername = os.path.basename(os.path.dirname(self.selected_image_path))
            self._widget.label4.setText(foldername)

            # Update Image
            self._show_selected_image()
        

    def _show_selected_image(self):

        # Load Image
        img = cv2.imread(self.selected_image_path)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        h,w,channels = img.shape
        bytesPerLine = channels * w

        # Convert Image Datatype
        qImg = QImage(img.data, w, h, bytesPerLine, QImage.Format_RGB888)
        pixmap_image = QPixmap(QPixmap.fromImage(qImg))

        # Update Image
        self._widget.selected_image.setPixmap(pixmap_image)
        self._widget.selected_image.setAlignment(Qt.AlignCenter)
        self._widget.selected_image.setScaledContents(True)
        self._widget.selected_image.show()

        # Update On-Screen Image Stats
        self._widget.label5.setText("Shape: " + str((h,w)))


    def on_t1_initialize_clicked(self):
        """Equivelent to running roslaunch light_painting t1_initialize.launch"""

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        lfile = os.path.join(rospkg.RosPack().get_path('light_painting'), 'launch', 't1_initialize.launch')
        launch = roslaunch.parent.ROSLaunchParent(uuid, [lfile])
        launch.start()

        rospy.loginfo("LightPaintingGui started t1_initialize.launch")


    def on_start_program_clicked(self):
        """Equivelent to running roslaunch light_painting t1_execute.launch"""

        launch_file_path = os.path.join(rospkg.RosPack().get_path('light_painting'), 'launch', 't1_execute.launch')

        # Setup Launch
        self.myLauncher = roslaunch.scriptapi.ROSLaunch()
        myNode = roslaunch.core.Node(package='light_painting', node_type='lightPainter.py', name="PainterNode", respawn="false")

        # Launch new node
        self.myLauncher.start()    
        self.myLauncher.launch(node=myNode)

        rospy.loginfo("LightPaintingGui started t1_execute.launch")


    def on_stop_program_clicked(self):
        """Stop ROS Node: Monet"""

        self.myLauncher.stop()
        rospy.loginfo("LightPaintingGui killed rosnode MONET")