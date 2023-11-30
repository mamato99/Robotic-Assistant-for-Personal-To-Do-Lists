# Cognitive Robotics

## ROS Setup

Installazione di ROS

```bash
# Setup sources.list and keys
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl -y # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Installation and environment setup
sudo apt update
sudo apt install ros-noetic-desktop-full
sudo source /opt/ros/noetic/setup.bash
sudo echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# To use with python3
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
sudo apt install python3-rosdep -y
sudo apt-get install python3-catkin-tools -y
sudo rosdep init
rosdep update
```



## RASA Setup

Installazione di RASA

```bash
sudo apt update
sudo apt install python3-pip
python3 -m pip install pip==22.0.0
python3 -m pip install pyOpenSSL
python3 –m pip install rasa==2.7.2
python3 –m pip install rasa[spacy]
python3 –m pip install rasa[transformers]
```

Installazione del modello utilizzato

```
python3 -m spacy download en_core_web_md
```



### Duckling

Duckling è un extractor per RASA da noi utilizzato per estrarre entità temporali come la deadline. Per maggiori informazioni rimandiamo alla documentazione ufficiale [qui](https://rasa.com/docs/rasa/2.x/components#ducklingentityextractor).

Seguire i seguenti comandi per la configurazione. Attenersi ai path riportati.

```bash
# Dependecies for Duckling
wget -qO- https://get.haskellstack.org/ | sh 
sudo apt install libicu-dev
sudo apt install libpcre3-dev

# Clone repository
cd cogrob-todolist/src/rasa_ros/chatbot 
git clone https://github.com/facebook/duckling
cd cogrob-todolist/src/rasa_ros/chatbot/duckling
stack build
```



## Tablet Setup

### Flask Server

Flask è un Web Server in esecuzione sulla rete locale utilizzato per la visualizzazione di attività e categorie di un utente su una pagina HTML. Per maggiori informazioni rimandiamo alla documentazione ufficiale [qui](https://flask.palletsprojects.com/en/2.2.x/).

```
pip install flask
```



### Rosbridge 

Rosbridge fornisce un'API JSON alla funzionalità ROS per i programmi non ROS. Per maggiori informazioni rimandiamo alla documentazione ufficiale [qui](http://wiki.ros.org/rosbridge_suite).

```bash
sudo apt-get install ros-noetic-rosbridge-server
pip install roslibpy
```



## Audio Setup

```bash
sudo apt-get install libasound-dev ffmpeg portaudio19-dev libportaudio2 libportaudiocpp0
pip3 install --user pyaudio speechrecognition librosa sounddevice python_speech_features scipy
```



# Run project

Rendere gli script di lancio eseguibili se non dovessero esserlo e montare la workspace ROS

```bash
chmod u+x cogrob-todolist/src/*
catkin build
```

Lanciare il server Flask, assicurandosi che sia visibile nella rete locale.

```bash
python3 cogrob-todolist/src/tablet_pkg/flask_server/app.py
```

Prima di eseguire controlla il file `config.py`, impostando l'indice corretto del microfono con `MIC_INDEX` (`None` per usare il microfono di default). Se esegui con Pepper, assicurati che la variabile`PEPPER` sia a `True`.

A questo punto il setup è completo. Per lanciare il progetto, dalla workspace ROS eseguire i seguenti comandi:

```bash
cd cogrob-todolist # workspace ROS
source setup.sh
roslaunch rasa_ros pepper.launch flask_ip:= #your ip
```



## Run in debug mode

Se si hanno problemi è possibile seguire i file launch separatamente come segue. In un nuovo terminale eseguire i seguenti comandi: 

```bash
cd cogrob-todolist # workspace ROS
source setup.sh
roslaunch pepper_nodes pepper_bringup.launch
```

Con il server Flask in esecuzione, In un altro terminale eseguire:

```bash
cd cogrob-todolist # workspace ROS
source setup.sh
roslaunch tablet_pkg tablet.launch flask_ip:= #your ip
```

In un altro terminale: 

```bash
cd cogrob-todolist # workspace ROS
source setup.sh
roslaunch rasa_ros chatbot.launch
```

A questo punto tutti i servizi necessari sono in esecuzione.

In un nuovo terminale eseguire il codice seguente per avviare l'integrazione dei servizi:

```bash
cd cogrob-todolist # workspace ROS
source setup.sh
rosrun rasa_ros dialog_interface.py
```

Infine, in un nuovo terminale, il seguente codice per avviare microfono e servizi audio:

```bash
cd cogrob-todolist # workspace ROS
source setup.sh
roslaunch ros_audio_pkg audio.launch
```

**Se non si dispone di Pepper** è possibile eseguire il progetto in *debug mode*, impostando la variabile `PEPPER` a `False` in `config.py` ed evitando di lanciare `pepper_bringup.launch` e `tablet.launch`.



# Contacts - Team 6

| Student            | E-mail | Matricola |
| ------------------ | ------ | --------- |
| Mario Amato        | m.amato72@studenti.unisa.it | 0622701670 |
| Margherita Avitabile   | m.avitabile6@studenti.unisa.it | 0622701825 |
| Lucia Battipaglia| l.battipaglia6@studenti.unisa.it | 0622701758 |
| Francesco Sonnessa | f.sonnessa@studenti.unisa.it | 0622701672 |