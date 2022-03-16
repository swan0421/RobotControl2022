# [2022-1] Robot Control 실습


#### Gazebo Simulation for RoK-3.
본 패키지는 휴머노이드 로봇 RoK-3의 교육용 Gazebo Simulation 입니다.
이 문서는 패키지의 설명 문서이며, 구성은 다음과 같습니다.

* What to do before simulation

* Simulation Manual
  1. [Download](https://github.com/swan0421/RobotControl2022) and Setting RobotControl2022
  2. Libraries used in RobotControl2022 Package
  3. How to run RobotControl2022 package, **Please read section 2 before proceeding.**
----

## What to do before simulation 
1. [RoS-Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) install, link : http://wiki.ros.org/kinetic/Installation/Ubuntu
2. [Gazeobo-7](http://gazebosim.org/tutorials?cat=install&tut=install_ubuntu&ver=7.0) install, link : http://gazebosim.org/tutorials?cat=install&tut=install_ubuntu&ver=7.0
3. [Netbeans-IDE](https://netbeans.apache.org/download/index.html) install, link : https://netbeans.apache.org/download/index.html

> **Java install before Netbeans install**
> ``` js
> sudo add-apt-repository ppa:webupd8team/java
> sudo apt update
> sudo apt install openjdk-8-jdk
> javac -version
> ```

> **Netbeans-IDE install**

>민호 재작성 요망 :
현재, RCLAB은 Netbeans IDE-8.2를 쓰고 있지만, 8.2 ver는 인터넷 상에서 이제는 다운로드가 안되므로, https://netbeans.apache.org/download/index.html 에서 최신 버전 다운 및 적용 후 자세한 내용은 여기에 다시 정리 바람.

4. [GitHub](https://github.com/)에 미리 가입한 상태면, 해당 패키지를 공동 작업하는데 있어 도움이됩니다. 따라서, 가입을 희망합니다. 또한, `Token password`를 발급받기 바랍니다.

----
## Simulation Manual 
### 1.[Download](https://github.com/swan0421/RobotControl2022) and Setting RobotControl2022
1. [RobotControl2022 Repository](https://github.com/swan0421/RobotControl2022)에 접속, link : https://github.com/swan0421/RobotControl2022
2. 해당 Repository에 접속 후에, `Code ▼`라는 초록색 버튼이 있는데 클릭하여 URL 주소 (https:/~)을 복사하거나,`Download ZIP` 을 통해 해당 패키지를 다운 받습니다.
3. NetBeans의 `Team` > `Remote` > `clone` 을 누른후, `Repository URL`을 https://github.com/swan0421/RobotControl2022.git 으로 설정합니다. (만약, NetBeans에서 `Team` > `Remote` > `clone` 경로가 보이지 않는 경우, NetBeans 화면 좌측에 있는 Projects 패널에서 catkin_ws 를 클릭하면 보이며, 위의 경로는 git에 연동되었을 때 활성화되는 경로이므로 처음 연동하는 것이라면, Team > git > clone으로 해도 됨) User에는 GitHUB의 user_name을 쓰고, Password에는 GitHUB의 `Token password`를 입력한 후 NEXT를 누릅니다.다
4. Select Remote Branches를 `master*` 로 선택하고 Next를 누릅니다.

5. Parent Directory를 사용자의 `home/user_name/catkin_ws/src` 경로로 설정하고, Clone name을 사용자가 원하는 이름으로 설정하고, (참고 : Clone Name은 패키지에 관련된 이름으로 써서 다른 폴더들과 구별 지을 것) Checkout Branch는 `master*` 로 설정하고, Remote Name은 origin으로 설정한 후 Finish를 누릅니다.

6. 사용자의 catkin_ws/src 위치에 Step5에서 설정한 Clone Name 을 갖는 폴더가 있는지 확인하고, 폴더 내부에 패키지 구성 파일들(world 폴더, src 폴더, launch 폴더 등)과 model 폴더(=`rok3_model`)이 있는지 확인합니다.
 

7. `rok3_model` 폴더를 `HOME/.gazebo/models/` 폴더로 가져와서 시뮬레이션을 위한 파일 셋팅을 마무리합니다. (`.gazebo` 폴더가 보이지 않으면, `Ctrl+H` 를 눌러서 폴더 숨김 해제를 할 것)
         
8. 패키지를 컴파일하기 위해 Netbeans에서 터미널 창을 열거나 기본 터미널 창에서 `catkin_make`을 입력하여 컴파일을 진행합니다. (터미널 창이 안보인다면, Netbeans의 상단 `Winodow > IDE Tools > Termianl` 을 클릭)


### 2.Libraries used in RobotControl2022 Package

| Library | Description |
| ------ | ----------- |
| [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)   | Eigen is a C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms.|
| [RBDL](https://rbdl.github.io/) | RBDL library contains highly efficient code for both forward and inverse dynamics for kinematic chains and branched models. |

**Recommended to re-install RBDL**

RBDL의 재설치를 권장합니다. 사용자마다 `root` 계정 혹은 `user` 계정으로 하기 때문에, Build 하는 과정에서 문제가 발생할 수 있습니다. 따라서, 다음과 같이 재설치를 해주시기 바랍니다. 본 패키지를 `root` 계정에서 사용할 경우, 재설치가 필요없을 수 있습니다.

**RBDL Build**

1. `RobotControl2022/src/RBDL/addons/urdfreader` 폴더 내에 있는 `CMakeList.txt` 파일에 `include_directories`를 다음과 같이 추가해줍니다.
* Before : 
``` js
IF (DEFINED ENV{ROS_ROOT})
	MESSAGE (STATUS "ROS found: $ENV{ROS_ROOT}")
	find_package(catkin REQUIRED COMPONENTS urdf)
```
* After :
``` js
IF (DEFINED ENV{ROS_ROOT})
	MESSAGE (STATUS "ROS found: $ENV{ROS_ROOT}")
	find_package(catkin REQUIRED COMPONENTS urdf)
	include_directories(include ${catkin_INCLUDE_DIRS})
```

2. RBDL make & install (**build 폴더가 존재할 경우, 삭제**)
RBDL 폴더에서 터미널 창을 켜고 아래의 명령어를 입력함으로써, RBDL 재설치를 끝냅니다.
``` js
mkdir build 
cd build/ 
cmake -D CMAKE_BUILD_TYPE=Release ../
cmake -D RBDL_BUILD_ADDON_URDFREADER=true ../
make 
sudo make install
```

### 3.How to run RobotControl2022 package
#### **!! 시뮬레이션 실행 전에 확인 해야하거나 셋팅 !!**

* #### Setting for Fixed / Floating Dynamics

`HOME/.gazebo/models/rok3_model`폴더에 있는 `model.sdf`를 엽니다. 그리고 Fixed / Floating Dynamics을 위해 `<fixed to world>`의 joint를 다음과 같이 셋팅 합니다.

**Setting Floating Dynamics in `model.sdf`**
``` js
<?xml version="1.0"?>
<sdf version='1.6'>
  <model name='rok3_model'>
  <!--joint name="fixed to world" type="fixed">
      <parent>world</parent>
      <child>base_link</child>
    </joint-->
.
.
.
  </model>
</sdf>
```


**Setting Fixed Dynamics in `model.sdf`**
``` js
<?xml version="1.0"?>
<sdf version='1.6'>
  <model name='rok3_model'>
  <joint name="fixed to world" type="fixed">
      <parent>world</parent>
      <child>base_link</child>
    </joint>
.
.
.
  </model>
</sdf>
```

다음으로, `catkin_ws/src/RcLab-RoK3/worlds`폴더에 있는 `rok3.world`를 엽니다. 그리고 Fixed / Floating Dynamics을 위해 모델의 `<pose frame>`을 다음과 같이 셋팅 합니다.

**Setting Floating Dynamics in `rok3.world`**
``` js
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="rok3">
.
.
.
    <include>
      <uri>model://rok3_model</uri>
      <pose frame=''>0 0 0.947 0 0 0</pose>
      <plugin name="rok3_plugin" filename="librok3_pkgs.so"/> 
    </include>
  </world>
</sdf>
```


**Setting Fixed Dynamics in `rok3.world`**
``` js
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="rok3">
.
.
.
    <include>
      <uri>model://rok3_model</uri>
      <pose frame=''>0 0 1.947 0 0 0</pose>
      <plugin name="rok3_plugin" filename="librok3_pkgs.so"/> 
    </include>
  </world>
</sdf>
```


* #### Check `model.urdf` file path for using RBDL in `rok3_plugin.cc`
* `rok3_plugin.cc`는 Gazebo main code 이며, `/catkin_ws/src/RcLab-RoK3/src`에 있습니다.
* **그리고, `rok3_plugin.cc`에서 사용자는 반드시 `Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)`함수에서, 아래 코드 예시와 같이 `Addons::URDFReadFromFile()` 함수 안에 적용되어 있는 `rok3_model.urdf`의 경로를 확인해주시고, 틀린다면 바로잡아주시기 바랍니다.**

* **`rok3_model.urdf`는 `home/.gazebo/models//rok3_model/urdf` 폴더에 있으며, 파일 속성 확인을 통해 정확한 경로 확인하시기 바랍니다.**

**In `rok3_plugin.cc`**
``` js
void gazebo::rok3_plugin::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
{
    /*
     * Loading model data and initializing the system before simulation 
     */

    //* model.sdf file based model data input to [physics::ModelPtr model] for gazebo simulation
    model = _model;

    //* [physics::ModelPtr model] based model update
    GetJoints();



    //* RBDL API Version Check
    int version_test;
    version_test = rbdl_get_api_version();
    printf(C_GREEN "RBDL API version = %d\n" C_RESET, version_test);

    //* model.urdf file based model data input to [Model* rok3_model] for using RBDL
    Model* rok3_model = new Model();
    Addons::URDFReadFromFile("/root/.gazebo/models/rok3_model/urdf/rok3_model.urdf", rok3_model, true, true);
    //↑↑↑ Check File Path ↑↑↑
    nDoF = rok3_model->dof_count - 6; // Get degrees of freedom, except position and orientation of the robot
    joint = new ROBO_JOINT[nDoF]; // Generation joint variables struct

    //* initialize and setting for robot control in gazebo simulation
    SetJointPIDgain();


    //* setting for getting dt
    last_update_time = model->GetWorld()->GetSimTime();
    update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&rok3_plugin::UpdateAlgorithm, this));

}
```

**모든 준비 과정이 끝나면, 다음과 같은 명령어를 통해 시뮬레이션 실행**
터미널 창에
```
roslaunch rok3_study_pkgs rok3.launch
```


