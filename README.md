- __ВНИМАНИЕ!__ Репозиторий обнавлен!
- Добавлено: 
	- Задача для бакалавров
	- Задача для магистров
- Bagfix: исправлена проблема для nvidia (подробности в issue)
- Bagfig: !ТОЛЬКО ДЛЯ VIRTUALBOX! добавлен файл для обновления docker контейнера (установка)


# Инструкция по установке

## Технические требования
    - Linux Ubuntu 16/18
    - Интернет соединение (~5-6 Гб трафика)
    
## Установка для видеокарт NVIDIA (yaprofi_robotics_2020/docker/nvidia)

Установите/обновите драйвер. Инструкция по [ссылке](https://www.cyberciti.biz/faq/ubuntu-linux-install-nvidia-driver-latest-proprietary-driver/)

Для версии драйвера 435.21, достаточно выполнить в терминале
```bash
    cd ~/yaprofi_robotics_2020 && sudo bash install_nvidia.bash
```

Для других версий, необходимо открыть файл, выполнив в терминале
```bash
    gedit ~/yaprofi_robotics_2020/docker/nvidia/Dockerfile.nvidia
```

Заменить версию в строке на вашу
```docker
    ARG NVIDIA_DRIVER_VERSION=435.21
```

Сохранить файл и выполнить скрипт
```bash
    cd ~/yaprofi_robotics_2020 && sudo bash install_nvidia.bash
```

## Установка для любых других видеокарт (yaprofi_robotics_2020/docker/others)

Выполнить в терминале
```bash
    cd ~/yaprofi_robotics_2020 && sudo bash install_others.bash
```

## Проверка, что все работает

1. Если процедура установки прошла без ошибок, вы должны увидеть окно с вращающимеся шестернями. Если этого не произошло, вы можете написать о возникших трудностях на странице https://github.com/be2rlab/yaprofi_robotics_2020/issues (нужно нажать на кнопку New issue)

2. В зависимости от модели видеокарты, в директории `./docker/XXX/` доступны несколько скриптов для работы с docker контейнером. (XXX это nvidia, либо others)
	a) `run_docker.bash` - запускает docker контейнер и входит в терминал внутри контейнера
	b) `exec_docker.bash` - запускает дополнительный терминал внутри запущенного контейнера

3. Проверка. Запустить симулятор CoppeliaSim, выполнив команды
	```bash
	cd /opt/csim
	./coppeliaSim.sh
	```

## Задачи, которые можно сделать, чтобы потренироваться работать с ПО

0. Запустить контейнер, выполнив в терминале
	```bash
	cd ~/yaprofi_robotics_2020
	sudo ./run_docker.bash
	```

1. На примере скрипта `./docker/XXX/exec_docker.bash` написать
	a) скрипт для запуска `roscore`
	b) скрипт для запуска симулятора. Путь и файл для запуска симулятора CoppeliaSim внтутри контейнера
	
	```bash
	/opt/csim/coppeliaSim.sh
	```
	c) скрипт для запуска rqt_graph
	d) скрипт для запуска rviz

2. Запустить последовательно все скрипты.

3. В соответствии с уроками по ROS на [wiki.ros.org/ROS/Tutorials](http://wiki.ros.org/ROS/Tutorials) запустить `turtlesim` внутри контейнера.

    

