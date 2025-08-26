import {
  Box,
  VStack,
  HStack,
  Text,
  Button,
  Container,
  Modal,
  ModalOverlay,
  ModalContent,
  ModalHeader,
  ModalBody,
  ModalCloseButton,
  useDisclosure,
  SimpleGrid,
  Card,
  CardBody,
  Image,
  Badge,
  Stat,
  StatLabel,
  StatNumber,
  StatHelpText,
  Accordion,
  AccordionItem,
  AccordionButton,
  AccordionPanel,
  AccordionIcon,
  Icon,
} from "@chakra-ui/react";
import { useState } from "react";
import { FaRocket, FaCode, FaVideo, FaUsers, FaClock, FaCheck } from "react-icons/fa";

const robotTypes = [
  {
    id: "turtlebot",
    name: "TurtleBot3",
    description: "Advanced mobile robot platform designed for autonomous navigation, mapping, and path planning in dynamic environments",
    image: "🤖",
    features: ["Autonomous Navigation", "SLAM Mapping", "Obstacle Avoidance", "Path Planning", "Real-time Localization"]
  },
  {
    id: "arm",
    name: "Robot Arm",
    description: "Precision 6-DOF manipulator capable of complex pick-and-place operations, trajectory planning, and fine motor control",
    image: "🦾",
    features: ["6 Degrees of Freedom", "Precise Positioning", "Object Manipulation", "Trajectory Planning", "Force Control"]
  },
  {
    id: "hand",
    name: "Robot Hand",
    description: "Sophisticated dexterous gripper system for complex manipulation tasks requiring adaptive grasping and fine motor skills",
    image: "🤲",
    features: ["Multi-finger Grasping", "Force Feedback", "Adaptive Grip", "Tactile Sensing", "Precision Control"]
  }
];

const LandingPage = ({ onGetStarted }) => {
  const { isOpen, onOpen, onClose } = useDisclosure();
  const [selectedRobot, setSelectedRobot] = useState(null);

  const handleRobotClick = (robot) => {
    setSelectedRobot(robot);
    onOpen();
  };

  return (
    <Box 
      minH="100vh"
      bgGradient="linear(135deg, brand.900 0%, brand.800 25%, brand.700 50%, brand.600 75%, brand.500 100%)"
      position="relative"
      overflow="hidden"
    >
      {/* Background overlay for depth */}
      <Box
        position="absolute"
        top="0"
        left="0"
        right="0"
        bottom="0"
        bgGradient="radial(circle at 30% 20%, brand.600 0%, transparent 50%)"
        opacity="0.3"
        zIndex="0"
      />
      
      <Container maxW="7xl" py={20} position="relative" zIndex="1">
        <VStack spacing={20} align="center">
          {/* Enhanced Hero Section */}
          <VStack spacing={8} textAlign="center" maxW="4xl">
            <Box position="relative">
              <Text fontSize="8xl" mb={6} 
                filter="drop-shadow(0 0 20px rgba(26, 140, 255, 0.5))"
                animation="pulse 2s infinite"
              >
                🤖
              </Text>
              <Box
                position="absolute"
                top="50%"
                left="50%"
                transform="translate(-50%, -50%)"
                w="200px"
                h="200px"
                bgGradient="radial(circle, brand.400 0%, transparent 70%)"
                opacity="0.2"
                borderRadius="full"
                filter="blur(40px)"
                zIndex="-1"
              />
            </Box>
            
            <VStack spacing={4}>
              <Text 
                fontSize={{ base: "4xl", md: "6xl", lg: "7xl" }} 
                fontWeight="900" 
                color="white"
                bgGradient="linear(45deg, white, blue.200)"
                bgClip="text"
                textShadow="0 4px 20px rgba(255, 255, 255, 0.3)"
                lineHeight="1.1"
              >
                Robot Programming
              </Text>
              <Text 
                fontSize={{ base: "3xl", md: "5xl", lg: "6xl" }} 
                fontWeight="900" 
                color="blue.300"
                textShadow="0 2px 10px rgba(26, 140, 255, 0.5)"
              >
                Console
              </Text>
            </VStack>
            
            <VStack spacing={6} maxW="3xl">
              <Text fontSize={{ base: "xl", md: "2xl" }} color="blue.100" fontWeight="500" lineHeight="1.6">
                Transform your robotics knowledge into real-world skills with our cutting-edge simulation platform.
              </Text>
              <Text fontSize={{ base: "lg", md: "xl" }} color="gray.300" lineHeight="1.7">
                Book dedicated time slots to program authentic robot systems in high-fidelity simulations. 
                Write Python code using ROS (Robot Operating System), control advanced robotic platforms, 
                and witness your algorithms come to life through immersive real-time visualizations.
              </Text>
              <Text fontSize={{ base: "md", md: "lg" }} color="gray.400" lineHeight="1.6">
                Whether you're mastering autonomous navigation with TurtleBot3, perfecting manipulation 
                tasks with robotic arms, or exploring dexterous grasping with advanced hand systems - 
                our platform provides the tools and environment for hands-on robotics learning.
              </Text>
            </VStack>
            
            <HStack spacing={6} pt={6}>
              <Button
                size="xl"
                h="auto"
                py={6}
                px={12}
                fontSize="xl"
                fontWeight="bold"
                bgGradient="linear(135deg, blue.400, blue.600)"
                color="white"
                boxShadow="0 8px 32px rgba(26, 140, 255, 0.4)"
                _hover={{
                  bgGradient: "linear(135deg, blue.300, blue.500)",
                  boxShadow: "0 12px 48px rgba(26, 140, 255, 0.6)",
                  transform: "translateY(-3px)",
                }}
                onClick={onGetStarted}
                leftIcon={<FaRocket />}
              >
                Start Your Journey
              </Button>
              
              <Button
                size="xl"
                h="auto"
                py={6}
                px={10}
                fontSize="lg"
                variant="outline"
                borderColor="blue.300"
                color="blue.200"
                boxShadow="0 4px 20px rgba(255, 255, 255, 0.1)"
                _hover={{
                  bg: "whiteAlpha.100",
                  borderColor: "blue.200",
                  color: "white",
                  boxShadow: "0 6px 30px rgba(255, 255, 255, 0.2)",
                  transform: "translateY(-2px)",
                }}
              >
                Learn More
              </Button>
            </HStack>
          </VStack>

        {/* Enhanced How It Works */}
        <VStack spacing={12} w="full" maxW="6xl">
          <VStack spacing={4} textAlign="center">
            <Text 
              fontSize={{ base: "3xl", md: "4xl", lg: "5xl" }} 
              fontWeight="900" 
              color="white"
              textShadow="0 2px 10px rgba(255, 255, 255, 0.3)"
            >
              How It Works
            </Text>
            <Text fontSize="xl" color="gray.300" maxW="2xl">
              Get started with robot programming in three simple steps
            </Text>
          </VStack>
          
          <SimpleGrid columns={{ base: 1, md: 3 }} spacing={8} w="full">
            <Card 
              bg="rgba(0, 20, 41, 0.8)" 
              border="2px solid" 
              borderColor="blue.400"
              boxShadow="0 16px 48px rgba(255, 255, 255, 0.15)"
              backdropFilter="blur(20px)"
              _hover={{
                boxShadow: "0 20px 60px rgba(255, 255, 255, 0.25)",
                transform: "translateY(-6px) scale(1.02)",
                borderColor: "blue.300",
              }}
              transition="all 0.4s ease"
            >
              <CardBody textAlign="center" py={10}>
                <Box mb={6} position="relative">
                  <Icon as={FaClock} fontSize="4xl" color="blue.300" mb={2} />
                  <Text 
                    fontWeight="bold" 
                    color="blue.200" 
                    bg="blue.900" 
                    px={3} 
                    py={1} 
                    borderRadius="full" 
                    display="inline-block"
                    position="absolute"
                    top="-10px"
                    right="-10px"
                    fontSize="sm"
                  >
                    01
                  </Text>
                </Box>
                <Text fontSize="2xl" fontWeight="bold" color="white" mb={4}>
                  Book a Time Slot
                </Text>
                <Text color="gray.300" fontSize="lg" lineHeight="1.6">
                  Choose from available time slots that fit your schedule. 
                  Select your preferred robot type and reserve dedicated simulation time.
                </Text>
              </CardBody>
            </Card>

            <Card 
              bg="rgba(0, 20, 41, 0.8)" 
              border="2px solid" 
              borderColor="blue.400"
              boxShadow="0 16px 48px rgba(255, 255, 255, 0.15)"
              backdropFilter="blur(20px)"
              _hover={{
                boxShadow: "0 20px 60px rgba(255, 255, 255, 0.25)",
                transform: "translateY(-6px) scale(1.02)",
                borderColor: "blue.300",
              }}
              transition="all 0.4s ease"
            >
              <CardBody textAlign="center" py={10}>
                <Box mb={6} position="relative">
                  <Icon as={FaCode} fontSize="4xl" color="blue.300" mb={2} />
                  <Text 
                    fontWeight="bold" 
                    color="blue.200" 
                    bg="blue.900" 
                    px={3} 
                    py={1} 
                    borderRadius="full" 
                    display="inline-block"
                    position="absolute"
                    top="-10px"
                    right="-10px"
                    fontSize="sm"
                  >
                    02
                  </Text>
                </Box>
                <Text fontSize="2xl" fontWeight="bold" color="white" mb={4}>
                  Write Python Code
                </Text>
                <Text color="gray.300" fontSize="lg" lineHeight="1.6">
                  Use our advanced Monaco editor with ROS integration. 
                  Write Python code with syntax highlighting, autocomplete, and real-time feedback.
                </Text>
              </CardBody>
            </Card>

            <Card 
              bg="rgba(0, 20, 41, 0.8)" 
              border="2px solid" 
              borderColor="blue.400"
              boxShadow="0 16px 48px rgba(255, 255, 255, 0.15)"
              backdropFilter="blur(20px)"
              _hover={{
                boxShadow: "0 20px 60px rgba(255, 255, 255, 0.25)",
                transform: "translateY(-6px) scale(1.02)",
                borderColor: "blue.300",
              }}
              transition="all 0.4s ease"
            >
              <CardBody textAlign="center" py={10}>
                <Box mb={6} position="relative">
                  <Icon as={FaVideo} fontSize="4xl" color="blue.300" mb={2} />
                  <Text 
                    fontWeight="bold" 
                    color="blue.200" 
                    bg="blue.900" 
                    px={3} 
                    py={1} 
                    borderRadius="full" 
                    display="inline-block"
                    position="absolute"
                    top="-10px"
                    right="-10px"
                    fontSize="sm"
                  >
                    03
                  </Text>
                </Box>
                <Text fontSize="2xl" fontWeight="bold" color="white" mb={4}>
                  See Live Results
                </Text>
                <Text color="gray.300" fontSize="lg" lineHeight="1.6">
                  Watch your algorithms execute in high-fidelity Gazebo simulations. 
                  Get instant video feedback and performance analytics.
                </Text>
              </CardBody>
            </Card>
          </SimpleGrid>
        </VStack>

        {/* Enhanced Available Robots */}
        <VStack spacing={12} w="full" maxW="6xl">
          <VStack spacing={4} textAlign="center">
            <Text 
              fontSize={{ base: "3xl", md: "4xl", lg: "5xl" }} 
              fontWeight="900" 
              color="white"
              textShadow="0 2px 10px rgba(255, 255, 255, 0.3)"
            >
              Available Robots
            </Text>
            <Text fontSize="xl" color="gray.300" maxW="2xl">
              Choose from our advanced robotic platforms for your programming sessions
            </Text>
          </VStack>
          
          <SimpleGrid columns={{ base: 1, md: 3 }} spacing={8} w="full">
            {robotTypes.map((robot) => (
              <Card 
                key={robot.id} 
                bg="rgba(0, 20, 41, 0.9)" 
                border="2px solid" 
                borderColor="blue.500"
                boxShadow="0 20px 60px rgba(255, 255, 255, 0.15)"
                backdropFilter="blur(20px)"
                cursor="pointer"
                onClick={() => handleRobotClick(robot)}
                _hover={{ 
                  borderColor: "blue.300", 
                  transform: "translateY(-8px) scale(1.03)",
                  boxShadow: "0 25px 80px rgba(255, 255, 255, 0.25)",
                }}
                transition="all 0.4s ease"
                position="relative"
                overflow="hidden"
              >
                {/* Hover glow effect */}
                <Box
                  position="absolute"
                  top="0"
                  left="0"
                  right="0"
                  bottom="0"
                  bgGradient="radial(circle at center, blue.400 0%, transparent 70%)"
                  opacity="0"
                  _groupHover={{ opacity: "0.1" }}
                  transition="opacity 0.4s ease"
                  zIndex="0"
                />
                
                <CardBody textAlign="center" py={10} position="relative" zIndex="1">
                  <Box mb={6} position="relative">
                    <Text 
                      fontSize="7xl" 
                      mb={4}
                      filter="drop-shadow(0 0 20px rgba(26, 140, 255, 0.5))"
                      transition="all 0.3s ease"
                      _groupHover={{ transform: "scale(1.1)" }}
                    >
                      {robot.image}
                    </Text>
                    <Box
                      position="absolute"
                      top="50%"
                      left="50%"
                      transform="translate(-50%, -50%)"
                      w="120px"
                      h="120px"
                      bgGradient="radial(circle, blue.400 0%, transparent 70%)"
                      opacity="0.3"
                      borderRadius="full"
                      filter="blur(30px)"
                      zIndex="-1"
                    />
                  </Box>
                  
                  <Text fontSize="2xl" fontWeight="bold" color="white" mb={4}>
                    {robot.name}
                  </Text>
                  <Text color="gray.300" mb={6} fontSize="md" lineHeight="1.6">
                    {robot.description}
                  </Text>
                  
                  <VStack spacing={3}>
                    <Text fontSize="sm" color="blue.300" fontWeight="bold" textTransform="uppercase" letterSpacing="wide">
                      Key Features
                    </Text>
                    <VStack spacing={2}>
                      {robot.features.map((feature, index) => (
                        <Badge 
                          key={index} 
                          colorScheme="blue" 
                          variant="subtle"
                          px={3}
                          py={1}
                          borderRadius="full"
                          fontSize="xs"
                          fontWeight="bold"
                          bg="blue.900"
                          color="blue.200"
                          border="1px solid"
                          borderColor="blue.600"
                        >
                          {feature}
                        </Badge>
                      ))}
                    </VStack>
                  </VStack>
                  
                  <Button
                    mt={6}
                    size="md"
                    bgGradient="linear(135deg, blue.400, blue.600)"
                    color="white"
                    _hover={{
                      bgGradient: "linear(135deg, blue.300, blue.500)",
                    }}
                    leftIcon={<FaCheck />}
                  >
                    Select Robot
                  </Button>
                </CardBody>
              </Card>
            ))}
          </SimpleGrid>
        </VStack>

        </VStack>

        {/* Statistics Section */}
        <VStack spacing={12} w="full" maxW="6xl" py={16}>
          <VStack spacing={4} textAlign="center">
            <Text 
              fontSize={{ base: "3xl", md: "4xl", lg: "5xl" }} 
              fontWeight="900" 
              color="white"
              textShadow="0 2px 10px rgba(255, 255, 255, 0.3)"
            >
              Trusted by Thousands
            </Text>
            <Text fontSize="xl" color="gray.300" maxW="2xl">
              Join a growing community of robotics enthusiasts and professionals
            </Text>
          </VStack>
          
          <SimpleGrid columns={{ base: 2, md: 4 }} spacing={8} w="full">
            <Card 
              bg="rgba(0, 20, 41, 0.8)" 
              border="2px solid" 
              borderColor="blue.400"
              boxShadow="0 16px 48px rgba(255, 255, 255, 0.15)"
              backdropFilter="blur(20px)"
              textAlign="center"
              py={6}
            >
              <CardBody>
                <Stat textAlign="center">
                  <StatNumber fontSize="4xl" color="blue.300" fontWeight="bold">
                    10K+
                  </StatNumber>
                  <StatLabel color="white" fontSize="lg" fontWeight="semibold">
                    Active Users
                  </StatLabel>
                  <StatHelpText color="gray.400">
                    Programming robots daily
                  </StatHelpText>
                </Stat>
              </CardBody>
            </Card>
            
            <Card 
              bg="rgba(0, 20, 41, 0.8)" 
              border="2px solid" 
              borderColor="blue.400"
              boxShadow="0 16px 48px rgba(255, 255, 255, 0.15)"
              backdropFilter="blur(20px)"
              textAlign="center"
              py={6}
            >
              <CardBody>
                <Stat textAlign="center">
                  <StatNumber fontSize="4xl" color="blue.300" fontWeight="bold">
                    50K+
                  </StatNumber>
                  <StatLabel color="white" fontSize="lg" fontWeight="semibold">
                    Code Sessions
                  </StatLabel>
                  <StatHelpText color="gray.400">
                    Successfully executed
                  </StatHelpText>
                </Stat>
              </CardBody>
            </Card>
            
            <Card 
              bg="rgba(0, 20, 41, 0.8)" 
              border="2px solid" 
              borderColor="blue.400"
              boxShadow="0 16px 48px rgba(255, 255, 255, 0.15)"
              backdropFilter="blur(20px)"
              textAlign="center"
              py={6}
            >
              <CardBody>
                <Stat textAlign="center">
                  <StatNumber fontSize="4xl" color="blue.300" fontWeight="bold">
                    99.9%
                  </StatNumber>
                  <StatLabel color="white" fontSize="lg" fontWeight="semibold">
                    Uptime
                  </StatLabel>
                  <StatHelpText color="gray.400">
                    Always available
                  </StatHelpText>
                </Stat>
              </CardBody>
            </Card>
            
            <Card 
              bg="rgba(0, 20, 41, 0.8)" 
              border="2px solid" 
              borderColor="blue.400"
              boxShadow="0 16px 48px rgba(255, 255, 255, 0.15)"
              backdropFilter="blur(20px)"
              textAlign="center"
              py={6}
            >
              <CardBody>
                <Stat textAlign="center">
                  <StatNumber fontSize="4xl" color="blue.300" fontWeight="bold">
                    24/7
                  </StatNumber>
                  <StatLabel color="white" fontSize="lg" fontWeight="semibold">
                    Support
                  </StatLabel>
                  <StatHelpText color="gray.400">
                    Expert assistance
                  </StatHelpText>
                </Stat>
              </CardBody>
            </Card>
          </SimpleGrid>
        </VStack>

        {/* Benefits Section */}
        <VStack spacing={12} w="full" maxW="6xl" py={16}>
          <VStack spacing={4} textAlign="center">
            <Text 
              fontSize={{ base: "3xl", md: "4xl", lg: "5xl" }} 
              fontWeight="900" 
              color="white"
              textShadow="0 2px 10px rgba(255, 255, 255, 0.3)"
            >
              Why Choose Our Platform
            </Text>
            <Text fontSize="xl" color="gray.300" maxW="2xl">
              Experience the future of robotics education and development
            </Text>
          </VStack>
          
          <SimpleGrid columns={{ base: 1, md: 2 }} spacing={8} w="full">
            <Card 
              bg="rgba(0, 20, 41, 0.8)" 
              border="2px solid" 
              borderColor="blue.400"
              boxShadow="0 16px 48px rgba(255, 255, 255, 0.15)"
              backdropFilter="blur(20px)"
              p={8}
            >
              <CardBody>
                <HStack spacing={4} align="start">
                  <Icon as={FaUsers} fontSize="3xl" color="blue.300" mt={1} />
                  <VStack align="start" spacing={3}>
                    <Text fontSize="xl" fontWeight="bold" color="white">
                      Expert Community
                    </Text>
                    <Text color="gray.300" lineHeight="1.6">
                      Join thousands of robotics engineers, researchers, and students. 
                      Share knowledge, collaborate on projects, and learn from industry experts.
                    </Text>
                  </VStack>
                </HStack>
              </CardBody>
            </Card>
            
            <Card 
              bg="rgba(0, 20, 41, 0.8)" 
              border="2px solid" 
              borderColor="blue.400"
              boxShadow="0 16px 48px rgba(255, 255, 255, 0.15)"
              backdropFilter="blur(20px)"
              p={8}
            >
              <CardBody>
                <HStack spacing={4} align="start">
                  <Icon as={FaRocket} fontSize="3xl" color="blue.300" mt={1} />
                  <VStack align="start" spacing={3}>
                    <Text fontSize="xl" fontWeight="bold" color="white">
                      Cutting-Edge Technology
                    </Text>
                    <Text color="gray.300" lineHeight="1.6">
                      Access the latest robotics frameworks including ROS Noetic, Gazebo simulation, 
                      and state-of-the-art robot models with realistic physics.
                    </Text>
                  </VStack>
                </HStack>
              </CardBody>
            </Card>
            
            <Card 
              bg="rgba(0, 20, 41, 0.8)" 
              border="2px solid" 
              borderColor="blue.400"
              boxShadow="0 16px 48px rgba(255, 255, 255, 0.15)"
              backdropFilter="blur(20px)"
              p={8}
            >
              <CardBody>
                <HStack spacing={4} align="start">
                  <Icon as={FaCheck} fontSize="3xl" color="blue.300" mt={1} />
                  <VStack align="start" spacing={3}>
                    <Text fontSize="xl" fontWeight="bold" color="white">
                      Hands-On Learning
                    </Text>
                    <Text color="gray.300" lineHeight="1.6">
                      Move beyond theory with practical, project-based learning. 
                      Build real skills through interactive programming and immediate feedback.
                    </Text>
                  </VStack>
                </HStack>
              </CardBody>
            </Card>
            
            <Card 
              bg="rgba(0, 20, 41, 0.8)" 
              border="2px solid" 
              borderColor="blue.400"
              boxShadow="0 16px 48px rgba(255, 255, 255, 0.15)"
              backdropFilter="blur(20px)"
              p={8}
            >
              <CardBody>
                <HStack spacing={4} align="start">
                  <Icon as={FaClock} fontSize="3xl" color="blue.300" mt={1} />
                  <VStack align="start" spacing={3}>
                    <Text fontSize="xl" fontWeight="bold" color="white">
                      Flexible Scheduling
                    </Text>
                    <Text color="gray.300" lineHeight="1.6">
                      Book sessions that fit your schedule. Available 24/7 with instant access 
                      to high-performance computing resources and simulation environments.
                    </Text>
                  </VStack>
                </HStack>
              </CardBody>
            </Card>
          </SimpleGrid>
        </VStack>

        {/* FAQ Section */}
        <VStack spacing={12} w="full" maxW="4xl" py={16}>
          <VStack spacing={4} textAlign="center">
            <Text 
              fontSize={{ base: "3xl", md: "4xl", lg: "5xl" }} 
              fontWeight="900" 
              color="white"
              textShadow="0 2px 10px rgba(255, 255, 255, 0.3)"
            >
              Frequently Asked Questions
            </Text>
            <Text fontSize="xl" color="gray.300" maxW="2xl">
              Everything you need to know about our robotics platform
            </Text>
          </VStack>
          
          <Card 
            bg="rgba(0, 20, 41, 0.8)" 
            border="2px solid" 
            borderColor="blue.400"
            boxShadow="0 16px 48px rgba(255, 255, 255, 0.15)"
            backdropFilter="blur(20px)"
            w="full"
          >
            <CardBody p={6}>
              <Accordion allowMultiple>
                <AccordionItem border="none" py={2}>
                  <AccordionButton 
                    _hover={{ bg: "whiteAlpha.100" }} 
                    borderRadius="md" 
                    py={4}
                  >
                    <Box flex="1" textAlign="left">
                      <Text fontSize="lg" fontWeight="bold" color="white">
                        What programming languages are supported?
                      </Text>
                    </Box>
                    <AccordionIcon color="blue.300" />
                  </AccordionButton>
                  <AccordionPanel pb={4} color="gray.300" fontSize="md" lineHeight="1.6">
                    Our platform primarily supports Python with ROS (Robot Operating System) integration. 
                    You'll have access to comprehensive ROS libraries, including navigation, manipulation, 
                    perception, and control packages.
                  </AccordionPanel>
                </AccordionItem>

                <AccordionItem border="none" py={2}>
                  <AccordionButton 
                    _hover={{ bg: "whiteAlpha.100" }} 
                    borderRadius="md" 
                    py={4}
                  >
                    <Box flex="1" textAlign="left">
                      <Text fontSize="lg" fontWeight="bold" color="white">
                        How realistic are the robot simulations?
                      </Text>
                    </Box>
                    <AccordionIcon color="blue.300" />
                  </AccordionButton>
                  <AccordionPanel pb={4} color="gray.300" fontSize="md" lineHeight="1.6">
                    Our simulations use Gazebo with high-fidelity physics engines, realistic sensor models, 
                    and accurate robot dynamics. The virtual environments closely mirror real-world conditions, 
                    making the transition to physical robots seamless.
                  </AccordionPanel>
                </AccordionItem>

                <AccordionItem border="none" py={2}>
                  <AccordionButton 
                    _hover={{ bg: "whiteAlpha.100" }} 
                    borderRadius="md" 
                    py={4}
                  >
                    <Box flex="1" textAlign="left">
                      <Text fontSize="lg" fontWeight="bold" color="white">
                        Can I save and share my code?
                      </Text>
                    </Box>
                    <AccordionIcon color="blue.300" />
                  </AccordionButton>
                  <AccordionPanel pb={4} color="gray.300" fontSize="md" lineHeight="1.6">
                    Yes! Your code is automatically saved during each session. You can download your projects, 
                    share them with colleagues, and build a portfolio of your robotics programming work.
                  </AccordionPanel>
                </AccordionItem>

                <AccordionItem border="none" py={2}>
                  <AccordionButton 
                    _hover={{ bg: "whiteAlpha.100" }} 
                    borderRadius="md" 
                    py={4}
                  >
                    <Box flex="1" textAlign="left">
                      <Text fontSize="lg" fontWeight="bold" color="white">
                        What if I'm new to robotics?
                      </Text>
                    </Box>
                    <AccordionIcon color="blue.300" />
                  </AccordionButton>
                  <AccordionPanel pb={4} color="gray.300" fontSize="md" lineHeight="1.6">
                    Perfect! Our platform includes beginner-friendly tutorials, example code snippets, 
                    and guided projects. Start with basic movement commands and gradually work up to 
                    complex autonomous behaviors.
                  </AccordionPanel>
                </AccordionItem>
              </Accordion>
            </CardBody>
          </Card>
        </VStack>

        {/* Enhanced Call to Action */}
        <VStack spacing={8} textAlign="center" py={20} maxW="4xl">
          <VStack spacing={6}>
            <Text 
              fontSize={{ base: "3xl", md: "4xl", lg: "5xl" }} 
              fontWeight="900" 
              color="white"
              textShadow="0 2px 10px rgba(255, 255, 255, 0.3)"
              lineHeight="1.2"
            >
              Ready to Transform Your 
            </Text>
            <Text 
              fontSize={{ base: "3xl", md: "4xl", lg: "5xl" }} 
              fontWeight="900" 
              bgGradient="linear(45deg, blue.300, blue.500)"
              bgClip="text"
              lineHeight="1.2"
            >
              Robotics Journey?
            </Text>
            <Text fontSize="xl" color="gray.300" maxW="2xl" lineHeight="1.6">
              Join thousands of engineers, researchers, and students who are already building the future with robots. 
              Start your first programming session today.
            </Text>
          </VStack>
          
          <HStack spacing={6} flexWrap="wrap" justify="center">
            <Button
              size="xl"
              h="auto"
              py={8}
              px={16}
              fontSize="xl"
              fontWeight="bold"
              bgGradient="linear(135deg, blue.400, blue.600)"
              color="white"
              boxShadow="0 12px 48px rgba(26, 140, 255, 0.4)"
              _hover={{
                bgGradient: "linear(135deg, blue.300, blue.500)",
                boxShadow: "0 16px 64px rgba(26, 140, 255, 0.6)",
                transform: "translateY(-4px)",
              }}
              onClick={onGetStarted}
              leftIcon={<FaRocket />}
            >
              Book Your Session Now
            </Button>
          </HStack>
          
          <Text fontSize="sm" color="gray.500">
            No credit card required • Free trial available • Cancel anytime
          </Text>
        </VStack>

        {/* Enhanced Robot Details Modal */}
        <Modal isOpen={isOpen} onClose={onClose} size="xl">
          <ModalOverlay backdropFilter="blur(10px)" />
          <ModalContent 
            bg="rgba(0, 20, 41, 0.95)" 
            color="white"
            border="2px solid"
            borderColor="blue.400"
            boxShadow="0 25px 80px rgba(255, 255, 255, 0.2)"
            backdropFilter="blur(20px)"
          >
            <ModalHeader>
              <HStack spacing={4}>
                <Text fontSize="4xl">{selectedRobot?.image}</Text>
                <VStack align="start" spacing={1}>
                  <Text fontSize="2xl" fontWeight="bold">{selectedRobot?.name}</Text>
                  <Text fontSize="sm" color="blue.300" fontWeight="semibold" textTransform="uppercase" letterSpacing="wide">
                    Advanced Robotics Platform
                  </Text>
                </VStack>
              </HStack>
            </ModalHeader>
            <ModalCloseButton 
              color="gray.400" 
              _hover={{ color: "white", bg: "whiteAlpha.200" }}
            />
            <ModalBody pb={8}>
              <VStack spacing={6} align="start">
                <Text color="gray.300" fontSize="lg" lineHeight="1.6">
                  {selectedRobot?.description}
                </Text>
                
                <Box w="full">
                  <Text fontWeight="bold" mb={4} fontSize="lg" color="blue.200">
                    Key Features & Capabilities:
                  </Text>
                  <SimpleGrid columns={1} spacing={3}>
                    {selectedRobot?.features.map((feature, index) => (
                      <HStack key={index} spacing={3} p={3} bg="whiteAlpha.50" borderRadius="md" border="1px solid" borderColor="blue.700">
                        <Icon as={FaCheck} color="blue.300" fontSize="sm" />
                        <Text fontSize="md" color="white">{feature}</Text>
                      </HStack>
                    ))}
                  </SimpleGrid>
                </Box>

                <Box w="full" pt={4}>
                  <Button
                    size="lg"
                    w="full"
                    h="auto"
                    py={4}
                    bgGradient="linear(135deg, blue.400, blue.600)"
                    color="white"
                    fontSize="lg"
                    fontWeight="bold"
                    boxShadow="0 8px 32px rgba(26, 140, 255, 0.4)"
                    _hover={{
                      bgGradient: "linear(135deg, blue.300, blue.500)",
                      boxShadow: "0 12px 48px rgba(26, 140, 255, 0.6)",
                      transform: "translateY(-2px)",
                    }}
                    onClick={() => {
                      onClose();
                      onGetStarted();
                    }}
                    leftIcon={<FaRocket />}
                  >
                    Start Programming This Robot
                  </Button>
                </Box>
              </VStack>
            </ModalBody>
          </ModalContent>
        </Modal>
      </Container>
    </Box>
  );
};

export default LandingPage;