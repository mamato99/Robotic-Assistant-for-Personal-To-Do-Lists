# Robotic Assistant for Personal To-Do Lists

## Project Overview

The goal of this project is to develop a robotic assistant to aid individuals in managing their personal to-do lists. The robotic platform will communicate with users using spoken natural language, utilizing a microphone to acquire the audio stream through PyAudio.

## Project Participants
- Amato Mario
- Sonnessa Francesco
- Avitabile Margherita
- Battipaglia Lucia

## Project Requirements

- The robotic assistant must communicate with users through spoken natural language.
- A microphone will be available on the robot to capture the audio stream through PyAudio.
- The robot should manage a distinct to-do list for each individual it interacts with.
  - If a new person is encountered, the robot will inquire about their name and create a new to-do list.
  - Person recognition can be achieved through either facial or voice recognition.

## To-Do List Requirements

Develop a ROS dialogue system that allows the user to insert and remove activities from a to-do list. The dialogue system should also enable the user to activate a reminder when the deadline is approaching.

Each element in the list is identified by a tag (identifying the activity), a deadline, and a category (e.g., CR course, sport, personal, etc) (*).

The dialogue system must allow the user to:
- View the activities in the to-do list
- Insert a new activity into the to-do list
- Remove an activity from the to-do list
- Manage multiple users
- (*) Manage multiple categories of to-do lists
- Update an activity in the to-do list

## Project Structure

The project is organized into modules addressing different aspects such as natural language processing, person recognition, and to-do list management. Each participant will contribute to specific components, ensuring a collaborative and well-structured development process.
