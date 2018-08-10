INFO
------------
This is an extension for [TestIt toolkit](https://github.com/GertKanter/testit) which allows using DTRON and UPPAAL timed automata for executing the tests.<br/>

SETUP
-----------
First, one has to check if [Docker](https://www.docker.com/) has been installed. Then you need to save instructions from docker directories with names “Dockerfile” and type “File”. You can either download it here (https://github.com/arturgummel/dtrontestit) or clone the repository. 

Finally, build the Testit base image by typing:<br/>
`$ docker build –no-cache -t testit:latest .` <br/>

To build SUT and testing images type next commands: <br/>
`$ docker build --no-cache -t testit_tb_sut .`<br/>
`$ docker build --no-cache -t testit_tb_testit .`<br/>

Look at usage examples in the tutorials directory!