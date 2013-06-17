from naoqi import ALProxy

IP   = "127.0.0.1"
PORT = 9559
ttsModule = ALProxy("ALTextToSpeech", IP , PORT)

ttsModule.say("pose");
ttsModule.sayToFile("pose","/var/volatile/pose.raw");
ttsModule.say("scripted pose");
ttsModule.sayToFile("scripted pose","/var/volatile/scripted_pose.raw");

