FROM tensorflow/tensorflow:latest-gpu-py3 

RUN pip install tensor2tensor
RUN pip install tensorflow-gpu
RUN pip install --upgrade pip
RUN pip install tensor2tensor[tensorflow_gpu]
RUN pip install tf-nightly-gpu
RUN pip install tf-nightly
RUN pip install tfds-nightly
RUN pip install tensorflow-probability
RUN pip install rsa
RUN pip install --upgrade rsa
RUN pip install httplib2

RUN apt-get update
RUN apt-get -y install git
RUN apt-get -y install vim
RUN apt-get -y install wget

WORKDIR "/"
RUN git clone https://github.com/RobotJustina/g2p-seq2seq
WORKDIR "/g2p-seq2seq"
RUN python3 setup.py install

RUN sed -i 's/input_modality*/modality/' /usr/local/lib/python3.5/dist-packages/g2p_seq2seq-6.2.2a0-py3.5.egg/g2p_seq2seq/g2p.py
#RUN sed -i 's/shard_to_cpu=params.locally_shard_to_cpu/#shard_to_cpu=params.locally_shard_to_cpu/' /usr/local/lib/python3.5/dist-packages/g2p_seq2seq-6.2.2a0-py3.5.egg/g2p_seq2seq/g2p_trainer_utils.py
#RUN sed -i 's/trainer_lib.create_run_config(/trainer_lib.create_run_config("my_model",/' /usr/local/lib/python3.5/dist-packages/g2p_seq2seq-6.2.2a0-py3.5.egg/g2p_seq2seq/g2p_trainer_utils.py
#RUN sed -i 's/eval_early_stopping_metric_minimize=True,/eval_early_stopping_metric_minimize=True,\n                      warm_start_from=None,/' /usr/local/lib/python3.5/dist-packages/g2p_seq2seq-6.2.2a0-py3.5.egg/g2p_seq2seq/g2p_trainer_utils.py
#RUN sed -i 's/hparams.add_hparam("schedule", schedule)/hparams.add_hparam("schedule", schedule)\n  hparams.add_hparam("warm_start_from", warm_start_from)/' /usr/local/lib/python3.5/dist-packages/g2p_seq2seq-6.2.2a0-py3.5.egg/g2p_seq2seq/g2p_trainer_utils.py 
#RUN sed -i 's/early_stopping_kwargs=early_stopping_kwargs)/early_stopping_kwargs=early_stopping_kwargs)\n  hook_context = trainer_lib.HookContext(estimator=estimator, problem=problem, hparams=hparams)/' /usr/local/lib/python3.5/dist-packages/g2p_seq2seq-6.2.2a0-py3.5.egg/g2p_seq2seq/g2p_trainer_utils.py
#RUN sed -i 's/T2TModel.get_train_hooks(model_name)/T2TModel.get_train_hooks(model_name, hook_context)/' /usr/local/lib/python3.5/dist-packages/g2p_seq2seq-6.2.2a0-py3.5.egg/g2p_seq2seq/g2p_trainer_utils.py
#RUN sed -i 's/T2TModel.get_eval_hooks(model_name)/T2TModel.get_eval_hooks(model_name, hook_context)/' /usr/local/lib/python3.5/dist-packages/g2p_seq2seq-6.2.2a0-py3.5.egg/g2p_seq2seq/g2p_trainer_utils.py

WORKDIR "/"
RUN wget -O g2p-seq2seq-cmudict.tar.gz https://sourceforge.net/projects/cmusphinx/files/G2P%20Models/g2p-seq2seq-model-6.2-cmudict-nostress.tar.gz/download
RUN tar xf g2p-seq2seq-cmudict.tar.gz

#RUN sed -i 's/GPU=0*/GPU=1/' Makefile \
#    && sed -i 's/OPENCV=0/OPENCV=1/' Makefile \
#    && sed -i 's/CUDNN=0/CUDNN=1/' Makefile \
#    && sed -i 's/\/usr\/local\/cuda\/lib64/\/usr\/local\/cuda\/lib64 -L\/usr\/local\/cuda-9.0\/targets\/x86_64-linux\/lib\/stubs/' Makefile
