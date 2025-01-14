# Cirobb

## 🚀 Overview

**Cirobb** is a **2D Rigid Body Physics Engine** inspired by **Box2D Lite** that was created with the purpose of helping people who are starting in this wonderful world of physics for Videogames. The Engine uses the Erin Catto's Contact Persistence Algorithm and it's implementation of the PGS (Projected Gauss Seidel) solver to solve the MLCP very well known as a SI (Sequential Impulse).

Additionally, it features a straightforward 2D Collision Detection system that efficiently calculates Contact Points, Normal Direction, and Penetration depth. This project aims to provide a clear and detailed understanding of how physics engines work under the hood, making it an ideal learning resource for game developers.

> 💡 For more complex simulations with multiple constraints and shapes, we recommend using **Box2D**, which is considered the best open-source 2D physics engine available.

## 📸 Captures

![Physics simulation with circles and rectangles](/images/img1.gif?raw=true)
![Advanced physics interactions](/images/img3.gif?raw=true)

## ⚡ Demos & Examples

- [WebAssembly Demo (v1.1.8)](https://jonpena.github.io/Cirobb-wasm)
- [YouTube Demo Video (v1.1.4)](https://youtu.be/j2p6qmOVA7M)

## 🔜 Future Features

<table>
  <tr>
    <td>✨ Direct Solver</td>
    <td>Improve Velocity Solver convergence</td>
  </tr>
  <tr>
    <td>🔗 Distance Constraint</td>
    <td>Optional feature for distance-based constraints</td>
  </tr>
  <tr>
    <td>🔷 Convex Polygons</td>
    <td>Collision Detection against convex polygons</td>
  </tr>
  <tr>
    <td>🔄 Contact Points</td>
    <td>Change from Distance-Based to Feature-Based</td>
  </tr>
</table>

## 📚 Learning Resources

### Experts and Their Contributions

#### Erin Catto

- Iterative Dynamics with Temporal Coherence
- Modeling and Solving Constraints
- Soft Constraints
- Numerical Methods
- Continuous Collision Detection
- Understanding Constraints

#### Dirk Gregorius

- The Separating Axis test Between Convex Polyhedra
- Robust Contact Creation for physics Simulation

#### Erwin Coumans

- Forum Bullet
- Exploring MLCP Solvers And Featherstone

#### Tonge Richard

- Iterative Rigid Body Solvers 2012
- Iterative Rigid Body Solvers 2013

#### Brian Vincent Mirtich

- Impulse-based Dynamic Simulation of Rigid Body Systems

#### Kenny Erleben

- Book of Physics-Based Animation 2005
- Numerical Methods for Linear Complementarity Problems in Physics-based Animation

#### David Baraff

- Analytical Methods for Dynamic Simulation of Non-penetrating Rigid Bodies
- Fast Contact Force Computation for Nonpenetrating Rigid Bodies
- Linear-Time Dynamics using Lagrange Multipliers
- Physically Based Modeling, Pixar Animation Studios

#### Jim Van Verth

- Numerical Integration
- Understanding Rotations

#### Matthias Müller

- Position Based Dynamics

#### Michael B. Cline

- Post-Stabilization for Rigid Body Simulation with Contact and Constraints

#### Randy Gaul

- Separating Axis Test (SAT) and Support Points in 2D

### Books on Game Physics and Collision Detection

- Game Physics Pearls by Gino van den Bergen and Dirk Gregorius
- Real-Time Collision Detection by Christer Ericson

## ⭐ Support the Project

<div align="center">

### If you find this project helpful, please give it a star! ⭐

<p align="center">
  <a href="https://github.com/jonpena/Cirobb">
    <img src="https://raw.githubusercontent.com/Tarikul-Islam-Anik/Microsoft-Teams-Animated-Emojis/master/Emojis/Travel%20and%20places/Glowing%20Star.png" alt="Star this repo" width="100">
  </a>
</p>

[![GitHub stars](https://img.shields.io/github/stars/jonpena/Cirobb.svg?style=social&label=Star&maxAge=2592000)](https://github.com/jonpena/Cirobb/stargazers/)

</div>

## 📝 Note

Cirobb is designed as an educational introduction to 2D physics engines. It intentionally maintains simplicity to facilitate learning and understanding of fundamental concepts.
