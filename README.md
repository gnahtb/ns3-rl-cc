# ns3-rl-cc

Congestion control using deep reinforcement learning with ns3

## Description

This was a part of my QSIURP research project in summer 2024. We investigated the landscape of machine learning application to improve system performance as a whole and delved deeper into using deep reinforcement learning for congestion control in particular. Using ns3 for data collection was a direction that few prior work had picked up, despite its potential.

## Getting Started

### Dependencies

- Anaconda
- ns3

### Run the code

Create and activate the environment from the saved specs:

```
conda env create -f env.yml
conda activate qsiurp_env
```

Install [ns3-gym](https://github.com/tkn-tub/ns3-gym/) as instructed. Then clone this repo in the examples folder of opengym.

Start training by running the following command:

```
python train.py
```

Evaluate and plot the graphs by changing `LOG_DIR` and `MODEL_FILE` and run `eval.py`:

```
python eval.py
```

## License

This project is licensed under the MIT License - see `LICENSE` for details
