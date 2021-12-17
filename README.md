# Pretty Patties
ECE 470 Final Project

Details
-----
With the automation of a lot of manufacturing it is clear that repetitive tasks are soon to be completed by a robot. This is one of the most repetitive tasks is within the food industry. Thus, our group aims to create a robot that automates the process of making burgers. The customer will choose how many burgers they want, and what kind of bread and 'meat' they want. The robot will work with a conveyour belt system to achieve this.  


Options For 'Meat' and Bread

Meats: Chicken, Beef, Veggie
Bread: White Plain, Whole Wheat, Pretzel

The users can apply their own condiments to their liking. 


**Codebase**
-----
This repository includes code to run the UR3 robot on the simulation software Gazebo. The package.xml file is used to build the entire repository into a package so that ROS can run the code in it. In the scripts folder there are many different python files to run code on the UR3. These include seperate functions for forward/inverse kinematics, interfacing functions for ROS, UR3 gripper functions and a header file. The folder labeled Important Gazebo files include 3 folders that need to be placed in the src directory of the catkin_project. The most important one is the drivers folder which includes a different gazebo.launch file along with mutliple important mesh and urdf files for our model. To run this project you need to source then run "roslaunch" the ur3_driver and on a new terminal source and "rosrun" of the package. If you would like to see a repo of the entire catkin_project take a look at this link: https://github.com/JJStorm22/catkin_fp

Updates
-----
12/17/21 - The final project updates - all new camera and OpenCV implementation/code, multiple new urdf files for testing and for new food, ability to make up to 3 burgers in one run (limited on food depot availability), and much much more! (Many commits made to public repositoy above with entire catkin file for building).

11/17/21 - Gripper Funcionality added, Inverse Kinematics code added to the func file in scripts, Burger, Bun and Cheese models all created but not currently working in model (will be fixed soon)

10/15/21 - package.xml setup (able to be ran and used by ROS now), Readme Updated, basic scripts structure added.

Project Update 2
-----

General Project Updates: Before this project update, we talked to a few TAs about some of the issues we could have with our fast food robot. We received a lot of different feedback and it was all very dependent on how far we took this project. For example, we were told that using full scale gravity on the whole sandwich along with elastics will make Gazebo unstable in certain cases. Using what we learned as a guide, we started at the basics and moved on from there.

Burger 3D Model Files: Using prior knowledge of CAD software, we first designed 4 basic parts to make a simple, plain burger. Our first ingredients we used were two buns, a burger patty, and a slice of cheese. These items were designed and exported from Fusion 360 with textures that match those of real food (the best they could be). These objects were then uploaded into Gazebo using a mesh file (.dae or .stl). There were originally a few issues with the scaling of the mesh files but we found the <scale> command was useful for fixing it.

Fast Food World File: We learned that the best way to start manipulating our objects with our UR3 robot was to create a .world file that could be loaded by our gazebo.launch file. We created our first .world file by adding 2 tables that were in Gazebo along with the mesh burger ingredients we made. This file was then linked to the launch file when roslaunch is called. 

There is one issue that we have found where the textures of the mesh files are not loading into the world. They are currently showing up transparent but are still in the model explorer. We are working to fix this small bug now. 


Robot Movement: After getting the world file loaded correctly, we now could implement both forward and inverse kinematics to move objects in Gazebo. Using our lab code from lab 4 and 5, inverse kinematics was a very simple application. We did have to make a few adjustments to our project_exec file so we could move the robot. We implemented a simple function called “move_food” that is similar to the sequence used in the lab for “move_block.” The implementation can be seen at this youtube link: https://youtu.be/Ja0aNMC185E

Sensing: We implemented gripper sensing in the last checkpoint, but had issues actually using it for this checkpoint. It took some help from our fellow ECE 470 students on the discord server to figure out our gripper issue. A student pointed out to us that the max/min distances and force of the gripper could be adjusted in the UR3 URDF file, once we adjusted them we could grab and place objects! This can also be seen in the video above. We also added our code for using the camera, it is similar to the blob detection we used in lab 5. We haven’t used it to move the robot quite yet because we had issues with the mesh files being transparent instead of an actual color for the camera to detect. 

Next Steps: Our next goal is to fix our issue with the mesh files for our burger not loading in the world correctly. We can then manipulate those items with the robot and make our burger. We were worried about Gazebo not working well with our project because of gravity, but at the moment the gravity component of Gazebo has worked (so far). Once we are able to load our burger parts into the UR3 .world correctly we will only need to code the robot to fully prepare the sandwich. We are getting much closer to our goal of a functioning fast food robot. 

Conclusion: Overall, this checkpoint has been successful. Using peer coding, and troubleshooting we were able to reach milestone 2. 
  
Project Update 1
-----
General Project Updates: After getting some feedback from TA and experimenting with Gazebo, we have decided that the condiments aspect within the realm of this project might be a bit more difficult. Thus, we have decided to take a slightly different approach and omit that idea. So our robot's goal is to assemble a basic burger using the robot arm and series of conveyor belts. This should simplify the process within the simulator. 

Environment Setup: Our first step to achieve our goals for this checkpoint, was to set up the environment, and launch Gazebo. We first made a repository so that we could store all our project files. Using the lab packages as a guide, we made a package.xml file along with 3 python files in a scripts folder: project_exec.py, project_header.py, project_kinematics_func.py. 

Scripts Folder Files: 
Robot Movement: After achieving that, we wanted to be able to move our robot to the points we want. So far, we have been able to move the robot arm using forward kinematics. Using our lab 3 code as a guide, we implemented forward kinematics functions into a separate ‘func’ file.  The movement can be seen at the following: https://www.youtube.com/watch?v=8-Bx7udjONI Furthermore, we have been able to spawn in objects using a spawn file. An image of 3 blocks we spawned using a spawn file:

Sensing: The sensing we first experimented with was the gripper feedback, we were able to get feedback with our gripper if an object was grabbed or not. This will be very helpful with our project of grabbing food since we will need to know if an object is being held or not. 

Next Steps: Our next goal is to manipulate the objects and move the robot with inverse kinematics as well. We also need to use the camera as well, which we should be able to achieve within a few labs. Lastly, we will need to model all of the rest of the physical items our robot will need to be manipulating: burger, buns, cheese, etc. We will need to add these to our spawn file and load them into the world when we first start running Gazebo. 

Conclusion: Overall, this checkpoint has been successful. Using peer coding, and troubleshooting we were able to reach milestone 1 with ease.

  
Group Members
-----
Pouya Akbarzadeh

Jacob Betz 

Tracy Tang









ACADEMIC INTEGRITY
-----
Please review the University of Illinois Student Code before starting,
particularly all subsections of Article 1, Part 4 Academic Integrity and Procedure [here](https://studentcode.illinois.edu/article1/part4/1-401/).

**§ 1‑402 Academic Integrity Infractions**

(a).	Cheating. No student shall use or attempt to use in any academic exercise materials, information, study aids, or electronic data that the student knows or should know is unauthorized. Instructors are strongly encouraged to make in advance a clear statement of their policies and procedures concerning the use of shared study aids, examination files, and related materials and forms of assistance. Such advance notification is especially important in the case of take-home examinations. During any examination, students should assume that external assistance (e.g., books, notes, calculators, and communications with others) is prohibited unless specifically authorized by the Instructor. A violation of this section includes but is not limited to:

(1)	Allowing others to conduct research or prepare any work for a student without prior authorization from the Instructor, including using the services of commercial term paper companies. 

(2)	Submitting substantial portions of the same academic work for credit more than once or by more than one student without authorization from the Instructors to whom the work is being submitted. 

(3) Working with another person without authorization to satisfy an individual assignment.

(b) Plagiarism. No student shall represent the words, work, or ideas of another as his or her own in any academic endeavor. A violation of this section includes but is not limited to:

(1)	Copying: Submitting the work of another as one’s own. 

(2)	Direct Quotation: Every direct quotation must be identified by quotation marks or by appropriate indentation and must be promptly cited. Proper citation style for many academic departments is outlined in such manuals as the MLA Handbook or K.L. Turabian’s A Manual for Writers of Term Papers, Theses and Dissertations. These and similar publications are available in the University bookstore or library. The actual source from which cited information was obtained should be acknowledged.

(3)	Paraphrase: Prompt acknowledgment is required when material from another source is paraphrased or summarized in whole or in part. This is true even if the student’s words differ substantially from those of the source. A citation acknowledging only a directly quoted statement does not suffice as an acknowledgment of any preceding or succeeding paraphrased material. 

(4)	Borrowed Facts or Information: Information obtained in one’s reading or research that is not common knowledge must be acknowledged. Examples of common knowledge might include the names of leaders of prominent nations, basic scientific laws, etc. Materials that contribute only to one’s general understanding of the subject may be acknowledged in a bibliography and need not be immediately cited. One citation is usually sufficient to acknowledge indebtedness when a number of connected sentences in the paper draw their special information from one source.

(c) Fabrication. No student shall falsify or invent any information or citation in an academic endeavor. A violation of this section includes but is not limited to:

(1)	Using invented information in any laboratory experiment or other academic endeavor without notice to and authorization from the Instructor or examiner. It would be improper, for example, to analyze one sample in an experiment and covertly invent data based on that single experiment for several more required analyses. 

(2)	Altering the answers given for an exam after the examination has been graded. 

(3)	Providing false or misleading information for the purpose of gaining an academic advantage.

(d)	Facilitating Infractions of Academic Integrity. No student shall help or attempt to help another to commit an infraction of academic integrity, where one knows or should know that through one’s acts or omissions such an infraction may be facilitated. A violation of this section includes but is not limited to:

(1)	Allowing another to copy from one’s work. 

(2)	Taking an exam by proxy for someone else. This is an infraction of academic integrity on the part of both the student enrolled in the course and the proxy or substitute. 

(3)	Removing an examination or quiz from a classroom, faculty office, or other facility without authorization.

(e)	Bribes, Favors, and Threats. No student shall bribe or attempt to bribe, promise favors to or make threats against any person with the intent to affect a record of a grade or evaluation of academic performance. This includes conspiracy with another person who then takes the action on behalf of the student.

(f)	Academic Interference. No student shall tamper with, alter, circumvent, or destroy any educational material or resource in a manner that deprives any other student of fair access or reasonable use of that material or resource. 

(1)	Educational resources include but are not limited to computer facilities, electronic data, required/reserved readings, reference works, or other library materials. 

(2)	Academic interference also includes acts in which the student committing the infraction personally benefits from the interference, regardless of the effect on other students.

