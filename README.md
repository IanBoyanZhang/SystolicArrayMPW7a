# Caravel User Project

MPW 7 Systolic Array submission

Systolic Array is a classical architecture that is recently revitalized among Neural Network accelerator designs.

It is the heart of Google's TPUs and major workhorses of DSP engines.

In this project, we manually build a 3x3 matrix multiplier with Multiply-Accumulate Units that support two popular data formats used in modern machine learning or neural networks applications.

FP16 and int8 modes are runtime configurable through the Caravel SoC core. The design is focusing on clean coding for ease of understanding and proper modulation for future extension.
