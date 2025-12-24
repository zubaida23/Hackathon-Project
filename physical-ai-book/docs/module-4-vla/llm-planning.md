---
sidebar_position: 3
---

# LLM-Based Cognitive Planning

## Introduction

Large Language Models (LLMs) provide powerful cognitive planning capabilities for humanoid robots. This section explores how to integrate LLMs into the VLA system to enable high-level task planning and decision-making based on natural language commands.

## Planning Architecture

The LLM-based planning system consists of several components:

```
┌─────────────────────────────────────────┐
│        Natural Language Input         │
└─────────────────┬─────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────┐
│      LLM Task Planner (Cognitive)      │
├─────────────────────────────────────────┤
│  - Task decomposition                 │
│  - Action sequencing                  │
│  - Constraint checking                │
│  - Resource allocation                │
└─────────────────┬─────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────┐
│        Action Executor                │
├─────────────────────────────────────────┤
│  - ROS 2 action servers               │
│  - Robot control interfaces           │
│  - Sensor feedback processing         │
└─────────────────────────────────────────┘
```

## Setting Up LLM Integration

For privacy and reproducibility, we'll use open-source LLMs that can run locally:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import json
import re

class LLMPlanner(Node):
    def __init__(self):
        super().__init__('llm_planner')

        # Subscribe to high-level commands
        self.command_sub = self.create_subscription(
            String, '/high_level_command', self.command_callback, 10)

        # Publisher for action sequences
        self.action_pub = self.create_publisher(String, '/action_sequence', 10)

        # Publisher for robot poses
        self.pose_pub = self.create_publisher(Pose, '/target_pose', 10)

        self.get_logger().info('LLM Planner initialized')

    def command_callback(self, msg):
        """Process high-level command and generate action plan"""
        command = msg.data
        self.get_logger().info(f'Received command: {command}')

        # Generate plan using LLM
        plan = self.generate_plan(command)

        if plan:
            # Publish the action sequence
            plan_msg = String()
            plan_msg.data = json.dumps(plan)
            self.action_pub.publish(plan_msg)

            self.get_logger().info(f'Generated plan: {plan}')

    def generate_plan(self, command):
        """Generate action plan from natural language command"""
        # This is a simplified example using rule-based planning
        # In practice, this would use a real LLM with proper prompts

        # Example command: "Go to the kitchen and bring me a cup"
        plan = []

        if 'go to' in command.lower() or 'move to' in command.lower():
            # Extract destination
            dest_match = re.search(r'go to (.+)|move to (.+)', command.lower())
            if dest_match:
                destination = dest_match.group(1) or dest_match.group(2)
                plan.append({
                    'action': 'NAVIGATE_TO',
                    'destination': destination.upper().replace(' ', '_'),
                    'parameters': {}
                })

        if 'bring' in command.lower() or 'pick up' in command.lower() or 'grab' in command.lower():
            # Extract object to pick up
            obj_match = re.search(r'bring me (.+)|pick up (.+)|grab (.+)', command.lower())
            if obj_match:
                obj = obj_match.group(1) or obj_match.group(2) or obj_match.group(3)
                plan.append({
                    'action': 'PICK_UP_OBJECT',
                    'object': obj.upper().replace(' ', '_'),
                    'parameters': {}
                })

        if 'put' in command.lower() or 'place' in command.lower():
            # Extract placement location
            place_match = re.search(r'put (.+)|place (.+)', command.lower())
            if place_match:
                plan.append({
                    'action': 'PLACE_OBJECT',
                    'parameters': {}
                })

        # If no specific actions identified, try to infer from context
        if not plan:
            if 'dance' in command.lower():
                plan.append({
                    'action': 'PERFORM_ACTION_SEQUENCE',
                    'action_sequence': 'DANCE_ROUTINE',
                    'parameters': {}
                })
            elif 'wave' in command.lower():
                plan.append({
                    'action': 'WAVE',
                    'parameters': {}
                })

        return plan

def main(args=None):
    rclpy.init(args=args)
    llm_planner = LLMPlanner()

    try:
        rclpy.spin(llm_planner)
    except KeyboardInterrupt:
        pass
    finally:
        llm_planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced LLM Integration with Local Models

For more sophisticated planning, we can integrate with local LLMs like Ollama:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests
import json

class AdvancedLLMPlanner(Node):
    def __init__(self):
        super().__init__('advanced_llm_planner')

        # Subscribe to high-level commands
        self.command_sub = self.create_subscription(
            String, '/high_level_command', self.command_callback, 10)

        # Publisher for action sequences
        self.action_pub = self.create_publisher(String, '/action_sequence', 10)

        # Ollama server configuration
        self.ollama_url = 'http://localhost:11434/api/generate'
        self.model_name = 'llama2'  # or 'mistral', 'phi3', etc.

        # Test connection to Ollama
        try:
            response = requests.get('http://localhost:11434/api/tags')
            if response.status_code == 200:
                self.get_logger().info('Connected to Ollama server')
            else:
                self.get_logger().error('Failed to connect to Ollama server')
        except Exception as e:
            self.get_logger().error(f'Ollama connection error: {e}')

        self.get_logger().info('Advanced LLM Planner initialized')

    def command_callback(self, msg):
        """Process high-level command with LLM"""
        command = msg.data
        self.get_logger().info(f'Received command: {command}')

        # Generate plan using LLM
        plan = self.generate_plan_with_llm(command)

        if plan:
            # Publish the action sequence
            plan_msg = String()
            plan_msg.data = json.dumps(plan)
            self.action_pub.publish(plan_msg)

            self.get_logger().info(f'Generated plan: {plan}')

    def generate_plan_with_llm(self, command):
        """Generate action plan using local LLM"""
        # Define the prompt for the LLM
        prompt = f"""
        You are a robot task planner. Given the following command, break it down into a sequence of executable actions for a humanoid robot.

        Command: {command}

        Available actions:
        - NAVIGATE_TO: Move to a specific location
        - PICK_UP_OBJECT: Grasp an object
        - PLACE_OBJECT: Put down an object
        - PERFORM_ACTION: Execute a specific motion (e.g., wave, dance)
        - LOOK_AT: Turn head to look at something
        - SPEAK: Say something to the user
        - WAIT: Pause for a specific duration

        Please respond with a JSON list of actions with the following format:
        [
            {{
                "action": "NAVIGATE_TO",
                "parameters": {{"location": "KITCHEN"}},
                "description": "Move to the kitchen"
            }},
            {{
                "action": "PICK_UP_OBJECT",
                "parameters": {{"object": "CUP"}},
                "description": "Pick up the cup"
            }}
        ]

        Make sure the plan is feasible and includes all necessary steps to complete the task.
        """

        try:
            # Call Ollama API
            response = requests.post(
                self.ollama_url,
                json={
                    'model': self.model_name,
                    'prompt': prompt,
                    'stream': False
                }
            )

            if response.status_code == 200:
                result = response.json()
                response_text = result['response']

                # Extract JSON from the response
                json_match = re.search(r'\[.*\]', response_text, re.DOTALL)
                if json_match:
                    json_str = json_match.group(0)
                    plan = json.loads(json_str)
                    return plan
                else:
                    self.get_logger().error(f'Could not extract JSON from LLM response: {response_text}')
                    return None
            else:
                self.get_logger().error(f'LLM API request failed: {response.status_code}')
                return None

        except Exception as e:
            self.get_logger().error(f'LLM planning error: {e}')
            return None

def main(args=None):
    rclpy.init(args=args)
    advanced_planner = AdvancedLLMPlanner()

    try:
        rclpy.spin(advanced_planner)
    except KeyboardInterrupt:
        pass
    finally:
        advanced_planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Planning with World Knowledge

For more effective planning, the LLM needs to understand the robot's environment:

```python
class ContextAwarePlanner(Node):
    def __init__(self):
        super().__init__('context_aware_planner')

        # Subscribe to various sensors and location data
        self.command_sub = self.create_subscription(
            String, '/high_level_command', self.command_callback, 10)

        # Publisher for action sequences
        self.action_pub = self.create_publisher(String, '/action_sequence', 10)

        # Store world knowledge
        self.locations = {
            'KITCHEN': {'x': 1.0, 'y': 2.0},
            'LIVING_ROOM': {'x': 0.0, 'y': 0.0},
            'BEDROOM': {'x': -1.0, 'y': 2.0},
            'BATHROOM': {'x': -1.0, 'y': -1.0}
        }

        self.objects = {
            'CUP': {'location': 'KITCHEN', 'graspable': True},
            'BOOK': {'location': 'LIVING_ROOM', 'graspable': True},
            'CHAIR': {'location': 'LIVING_ROOM', 'graspable': False}
        }

        self.robot_state = {
            'current_location': 'LIVING_ROOM',
            'holding_object': None
        }

    def command_callback(self, msg):
        """Process command with context awareness"""
        command = msg.data
        self.get_logger().info(f'Received command: {command}')

        # Generate plan considering current context
        plan = self.generate_contextual_plan(command)

        if plan:
            # Publish the action sequence
            plan_msg = String()
            plan_msg.data = json.dumps(plan)
            self.action_pub.publish(plan_msg)

    def generate_contextual_plan(self, command):
        """Generate plan considering world state and knowledge"""
        # This is a simplified example
        # In practice, this would be handled by the LLM with full context

        plan = []

        # Add current state to the command context
        context = f"""
        Current robot state:
        - Location: {self.robot_state['current_location']}
        - Holding: {self.robot_state['holding_object'] or 'nothing'}

        Available locations: {list(self.locations.keys())}
        Available objects: {list(self.objects.keys())}
        """

        # Generate plan based on command and context
        if 'kitchen' in command.lower():
            plan.append({
                'action': 'NAVIGATE_TO',
                'parameters': {'location': 'KITCHEN'},
                'description': 'Move to kitchen'
            })

        if 'cup' in command.lower():
            if self.objects['CUP']['location'] != self.robot_state['current_location']:
                # Need to navigate to cup location first
                plan.append({
                    'action': 'NAVIGATE_TO',
                    'parameters': {'location': self.objects['CUP']['location']},
                    'description': f'Navigate to {self.objects["CUP"]["location"]} to get cup'
                })

            plan.append({
                'action': 'PICK_UP_OBJECT',
                'parameters': {'object': 'CUP'},
                'description': 'Pick up the cup'
            })

        return plan
```

## Plan Execution and Monitoring

The planning system should also monitor plan execution and adapt as needed:

```python
class PlanExecutor(Node):
    def __init__(self):
        super().__init__('plan_executor')

        # Subscribe to action sequences
        self.plan_sub = self.create_subscription(
            String, '/action_sequence', self.plan_callback, 10)

        # Publisher for individual actions
        self.action_pub = self.create_publisher(String, '/robot_action', 10)

        # Subscriber for action completion feedback
        self.feedback_sub = self.create_subscription(
            String, '/action_feedback', self.feedback_callback, 10)

        self.current_plan = []
        self.current_step = 0
        self.plan_active = False

    def plan_callback(self, msg):
        """Receive and start executing a plan"""
        try:
            plan = json.loads(msg.data)
            self.current_plan = plan
            self.current_step = 0
            self.plan_active = True

            self.get_logger().info(f'Starting plan execution with {len(plan)} steps')
            self.execute_next_step()

        except json.JSONDecodeError:
            self.get_logger().error('Invalid plan format received')

    def execute_next_step(self):
        """Execute the next step in the current plan"""
        if not self.plan_active or self.current_step >= len(self.current_plan):
            self.plan_active = False
            self.get_logger().info('Plan execution completed')
            return

        current_action = self.current_plan[self.current_step]
        self.get_logger().info(f'Executing step {self.current_step + 1}: {current_action["action"]}')

        # Publish the action
        action_msg = String()
        action_msg.data = json.dumps(current_action)
        self.action_pub.publish(action_msg)

    def feedback_callback(self, msg):
        """Handle action completion feedback"""
        feedback = json.loads(msg.data)

        if feedback.get('status') == 'SUCCESS':
            self.current_step += 1
            if self.current_step < len(self.current_plan):
                # Execute next step
                self.execute_next_step()
            else:
                self.get_logger().info('All plan steps completed successfully')
                self.plan_active = False
        elif feedback.get('status') == 'FAILURE':
            self.get_logger().error(f'Action failed: {feedback.get("error")}')
            # In a more sophisticated system, you might replan here
            self.plan_active = False
```

## Safety and Constraint Checking

LLMs should also consider safety constraints in their planning:

```python
def generate_safe_plan(self, command):
    """Generate plan with safety constraints"""
    safety_prompt = f"""
    Command: {command}

    Safety constraints to consider:
    1. Avoid obstacles and collisions
    2. Don't pick up dangerous objects (sharp, hot, heavy)
    3. Don't go to restricted areas
    4. Maintain stable walking gait
    5. Respect human personal space (minimum 1m distance)
    6. Don't attempt impossible actions

    Generate a safe action plan that considers these constraints.
    """

    # Use the safety_prompt with your LLM to generate a safe plan
    # Implementation would be similar to previous examples
    pass
```

## Conclusion

LLM-based cognitive planning enables humanoid robots to understand and execute complex natural language commands. By combining world knowledge, safety constraints, and plan monitoring, we create a robust system for high-level task execution. In the next section, we'll explore how to execute these plans as robotic actions.