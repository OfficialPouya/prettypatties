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

