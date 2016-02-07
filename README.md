# PID-Controller
My attempt at learning what a PID controller is.


## Dependencies
- numpy
- scipy
- matplotlib


### Installing
I ran into an error trying to install scipy in a requirements.txt. To get around this, I installed numpy and scipy separately before installing pylab.

```sh
$ virtualenv PID_venv
$ source PID_venv/bin/activate
$ pip install numpy==1.10.4
$ pip install scipy==0.17.0
$ pip install pylab==0.1.3
```


## Example usage
```sh
$ python test_simple_circuit.py
$ eog power.png
```

## Resources
- https://en.wikipedia.org/wiki/PID_controller#Manual_tuning (This is the best gif I have seen explaining how the different Ks affect the output.)
- https://pythonhosted.org/controlsystems/controlsystems-module.html
