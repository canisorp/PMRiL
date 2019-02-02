# Teleupravljanje mobilnom bazom i robotskim manipulatorom

Sklopovski povezana mobilna baza (paletar) s robotskim manipulatorom ([CPR Mover 4](https://cpr-robots.com/education#Mover4)) i implementirana mogućnost upravljanja s oba podsustava butem bežičnog *joysticka*. Ista mogućnost implementirana i putem web grafičkog sučelja koja ima mogućnost prikaz videa s web kamere potavljene na robotu. Upravljanje robotskim manipulatorom ostvareno je putem inverznog kinematičkog pristupa.

## Preduvjeti

### *Hardware*

* mobilna baza (paletar) - sa centralnim računalom koje radi na Linuxu te sa vlastitom lokalnom mrežom
* manipulator (robotska ruka) - povezana na 12V napajanje i na centralno računalo paletara USB-CAN komunikacijskim protokolom

### *Software*

* bilo koja distribucija Linuxa na računalu mobilne baze
* [ROS](http://www.ros.org/), u konkretnom slučaju [ROS kinetic](http://wiki.ros.org/kinetic) - fleksibilan *framework* za razvoj robotskog softvera
* [Peak-System CAN sučelje](http://www.peak-system.com/fileadmin/media/linux/index.htm) - osgirava mogućnost komunikacije robotske ruke s ROS-om putem PCAN-USB komunikacijskog protokola, za instalaciju i test rada CPR Mover 4 robotske ruke posjetiti službeni GitHub profil [CPR-Robots](https://github.com/CPR-Robots/Mover4) 
* Rosbridge_suite

## Razvoj

TBA

## Instalacija i pokretanje

TBA

## Autori

* Filip Bašić
* Joško Jukić
* Ante Lojić Kapetanović
* Ana Šćulac

za kolegij [Programiranje mobilnih robota](https://nastava.fesb.unist.hr/nastava/predmeti/9687)
