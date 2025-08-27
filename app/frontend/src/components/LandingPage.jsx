import {
  Box,
  VStack,
  HStack,
  Text,
  Button,
  Container,
  SimpleGrid,
  Card,
  CardBody,
  CardHeader,
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
  Input,
  Textarea,
  FormControl,
  FormLabel,
  Flex,
  Avatar,
  Divider,
  Link,
  Heading,
  useColorModeValue,
} from "@chakra-ui/react";
import { 
  FaRobot, 
  FaBrain, 
  FaGraduationCap, 
  FaChartLine, 
  FaCogs, 
  FaLightbulb,
  FaShieldAlt,
  FaRocket,
  FaPhone,
  FaEnvelope,
  FaMapMarkerAlt,
  FaFacebook,
  FaTwitter,
  FaLinkedin,
  FaGithub,
  FaStar,
  FaArrowRight,
  FaPlay,
  FaQuoteLeft,
  FaUsers,
  FaAward,
  FaGlobe,
  FaHandshake,
  FaCode,
  FaServer,
  FaCloud,
  FaChevronDown,
  FaCheck
} from "react-icons/fa";
import { useState } from "react";
import { submitContactMessage } from "../api";
import { useToast } from "@chakra-ui/react";

const LandingPage = ({ onGetStarted }) => {
  const [formData, setFormData] = useState({
    name: "",
    email: "",
    message: ""
  });
  const [isSubmitting, setIsSubmitting] = useState(false);
  const toast = useToast();

  const handleInputChange = (e) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value
    }));
  };

  const handleSubmitMessage = async () => {
    console.log("[v0] Contact form submission attempt");
    if (!formData.name || !formData.email || !formData.message) {
      toast({
        title: "Missing Information",
        description: "Please fill in all fields",
        status: "warning",
        duration: 3000,
        isClosable: true,
      });
      return;
    }

    setIsSubmitting(true);
    try {
      await submitContactMessage(formData);
      console.log("[v0] Contact form submitted successfully");
      toast({
        title: "Message Sent!",
        description: "Thank you for contacting us. We'll get back to you soon.",
        status: "success",
        duration: 5000,
        isClosable: true,
      });
      setFormData({ name: "", email: "", message: "" });
    } catch (error) {
      console.log("[v0] Contact form submission failed:", error);
      toast({
        title: "Error",
        description: "Failed to send message. Please try again.",
        status: "error",
        duration: 5000,
        isClosable: true,
      });
    } finally {
      setIsSubmitting(false);
    }
  };

  const handleGetStarted = () => {
    console.log("[v0] Get Started button clicked");
    onGetStarted();
  };

  const handleBookDemo = () => {
    console.log("[v0] Book Demo button clicked");
    // Could implement demo booking functionality
  };

  // Custom Robot Icons
  const RobotArmIcon = (props) => (
    <Box {...props}>
      <svg viewBox="0 0 24 24" fill="currentColor" width="100%" height="100%">
        <path d="M12 2C13.1 2 14 2.9 14 4V6H16C17.1 6 18 6.9 18 8V10H20C21.1 10 22 10.9 22 12C22 13.1 21.1 14 20 14H18V16C18 17.1 17.1 18 16 18H14V20C14 21.1 13.1 22 12 22C10.9 22 10 21.1 10 20V18H8C6.9 18 6 17.1 6 16V14H4C2.9 14 2 13.1 2 12C2 10.9 2.9 10 4 10H6V8C6 6.9 6.9 6 8 6H10V4C10 2.9 10.9 2 12 2Z"/>
      </svg>
    </Box>
  );

  const RobotCarIcon = (props) => (
    <Box {...props}>
      <svg viewBox="0 0 24 24" fill="currentColor" width="100%" height="100%">
        <path d="M18,16H16V14H18M12,16H10V14H12M6,16H4V14H6M18.5,9.5L16.38,7.38C15.63,6.63 14.63,6.25 13.63,6.25H10.38C9.38,6.25 8.38,6.63 7.63,7.38L5.5,9.5H3V16.5H4.5V18.5H6.5V16.5H17.5V18.5H19.5V16.5H21V9.5H18.5M17.5,10.5V13.5H15.5V10.5H17.5M8.5,10.5V13.5H6.5V10.5H8.5Z"/>
      </svg>
    </Box>
  );

  const TurtleBotIcon = (props) => (
    <Box {...props}>
      <svg viewBox="0 0 24 24" fill="currentColor" width="100%" height="100%">
        <path d="M12,2A3,3 0 0,1 15,5V7H18A1,1 0 0,1 19,8V10H20A1,1 0 0,1 21,11V13A1,1 0 0,1 20,14H19V16A1,1 0 0,1 18,17H15V19A3,3 0 0,1 12,22A3,3 0 0,1 9,19V17H6A1,1 0 0,1 5,16V14H4A1,1 0 0,1 3,13V11A1,1 0 0,1 4,10H5V8A1,1 0 0,1 6,7H9V5A3,3 0 0,1 12,2M8,8V16H16V8H8M10,10H14V14H10V10Z"/>
      </svg>
    </Box>
  );

  return (
    <Box 
      minH="100vh"
      bg="white"
      color="gray.800"
      position="relative"
      overflow="hidden"
    >
      {/* Sticky Header */}
      <Box
        position="sticky"
        top="0"
        zIndex="50"
        bg="primary.800"
        boxShadow="0 2px 10px rgba(0, 0, 0, 0.1)"
        borderBottom="1px solid"
        borderColor="gray.200"
      >
        <Container maxW="7xl">
          <Flex
            align="center"
            justify="space-between"
            py={4}
            px={4}
          >
            {/* Logo and Robot Icons */}
            <HStack spacing={8}>
              <Heading size="lg" color="white" fontWeight="700">
                RobotConsole
              </Heading>
              
              {/* Robot Icons */}
              <HStack spacing={4} display={{ base: "none", md: "flex" }}>
                <Box 
                  w={12} 
                  h={12} 
                  bg="secondary.500" 
                  borderRadius="lg" 
                  display="flex" 
                  align="center" 
                  justify="center"
                  color="white"
                >
                  <RobotArmIcon w={6} h={6} />
                </Box>
                <Box 
                  w={12} 
                  h={12} 
                  bg="accent.400" 
                  borderRadius="lg" 
                  display="flex" 
                  align="center" 
                  justify="center"
                  color="white"
                >
                  <RobotCarIcon w={6} h={6} />
                </Box>
                <Box 
                  w={12} 
                  h={12} 
                  bg="blue.500" 
                  borderRadius="lg" 
                  display="flex" 
                  align="center" 
                  justify="center"
                  color="white"
                >
                  <TurtleBotIcon w={6} h={6} />
                </Box>
              </HStack>
            </HStack>

            {/* Navigation and CTA */}
            <HStack spacing={6}>
              <HStack spacing={8} display={{ base: "none", lg: "flex" }}>
                <Link href="#features" color="gray.200" _hover={{ color: "white" }}>
                  Features
                </Link>
                <Link href="#case-studies" color="gray.200" _hover={{ color: "white" }}>
                  Case Studies
                </Link>
                <Link href="#contact" color="gray.200" _hover={{ color: "white" }}>
                  Contact
                </Link>
              </HStack>
              
              <HStack spacing={3}>
                <Button
                  variant="outline"
                  size="sm"
                  borderColor="gray.400"
                  color="gray.200"
                  _hover={{ borderColor: "white", color: "white" }}
                  onClick={handleBookDemo}
                >
                  Book Demo
                </Button>
                <Button
                  size="sm"
                  bgGradient="linear(to-r, cyan.400, blue.500)"
                  color="white"
                  _hover={{
                    bgGradient: "linear(to-r, cyan.300, blue.400)",
                    transform: "translateY(-1px)"
                  }}
                  onClick={handleGetStarted}
                >
                  Get Started
                </Button>
              </HStack>
            </HStack>
          </Flex>
        </Container>
      </Box>

      {/* Hero Section */}
      <Box
        position="relative"
        minH="90vh"
        bg="white"
        display="flex"
        align="center"
        py={20}
      >
        <Container maxW="7xl">
          <VStack spacing={16} w="full">
            <VStack spacing={8} textAlign="center" maxW="4xl">
              <Heading
                fontSize={{ base: "4xl", md: "5xl", lg: "6xl" }}
                fontWeight="900"
                color="gray.900"
                lineHeight="1.1"
              >
                Complete Development{" "}
                <Text as="span" bgGradient="linear(to-r, cyan.400, blue.500)" bgClip="text">
                  Workflow in Your Browser
                </Text>
              </Heading>
              
              <Text
                fontSize={{ base: "lg", md: "xl" }}
                color="gray.600"
                maxW="3xl"
                lineHeight="1.6"
              >
                Book your time slot, access the code editor, write ROS Python code, and see your robot 
                come to life in Gazebo simulation. Complete robotics development workflow with 
                real-time feedback and video output of your simulation results.
              </Text>
            </VStack>
            
            <HStack spacing={6} flexWrap="wrap" justify="center">
              <Button
                size="lg"
                px={8}
                py={6}
                fontSize="lg"
                fontWeight="600"
                bgGradient="linear(to-r, cyan.400, blue.500)"
                color="white"
                boxShadow="0 8px 32px rgba(6, 182, 212, 0.3)"
                _hover={{
                  bgGradient: "linear(to-r, cyan.300, blue.400)",
                  boxShadow: "0 12px 40px rgba(6, 182, 212, 0.4)",
                  transform: "translateY(-2px)",
                }}
                rightIcon={<FaArrowRight />}
                onClick={handleGetStarted}
              >
                Access Development Console
              </Button>
              
              <Button
                size="lg"
                px={8}
                py={6}
                fontSize="lg"
                fontWeight="600"
                variant="outline"
                borderColor="primary.800"
                color="primary.800"
                _hover={{
                  bg: "primary.50",
                  borderColor: "primary.900",
                }}
                leftIcon={<FaPlay />}
                onClick={handleBookDemo}
              >
                Watch Demo
              </Button>
            </HStack>
          </VStack>
        </Container>
      </Box>

      {/* Stats Section */}
      <Box bg="gray.50" py={20}>
        <Container maxW="7xl">
          <SimpleGrid columns={{ base: 1, md: 2, lg: 4 }} spacing={8} w="full">
            <VStack spacing={3} textAlign="center">
              <Text fontSize="4xl" fontWeight="700" color="primary.800">
                10k+
              </Text>
              <Text fontSize="lg" color="gray.600">
                Development Sessions
              </Text>
            </VStack>
            
            <VStack spacing={3} textAlign="center">
              <Text fontSize="4xl" fontWeight="700" color="primary.800">
                500+
              </Text>
              <Text fontSize="lg" color="gray.600">
                Active Developers
              </Text>
            </VStack>
            
            <VStack spacing={3} textAlign="center">
              <Text fontSize="4xl" fontWeight="700" color="primary.800">
                99.9%
              </Text>
              <Text fontSize="lg" color="gray.600">
                Uptime Guarantee
              </Text>
            </VStack>
            
            <VStack spacing={3} textAlign="center">
              <Text fontSize="4xl" fontWeight="700" color="primary.800">
                24/7
              </Text>
              <Text fontSize="lg" color="gray.600">
                Technical Support
              </Text>
            </VStack>
          </SimpleGrid>
        </Container>
      </Box>

      {/* Features Grid */}
      <Box py={20} id="features">
        <Container maxW="7xl">
          <VStack spacing={16} w="full">
            <VStack spacing={6} textAlign="center">
              <Heading
                fontSize={{ base: "3xl", md: "4xl", lg: "5xl" }}
                fontWeight="900"
                color="gray.900"
                lineHeight="1.2"
              >
                Professional Development{" "}
                <Text as="span" bgGradient="linear(to-r, cyan.400, blue.500)" bgClip="text">
                  Console Features
                </Text>
              </Heading>
              <Text
                fontSize="lg"
                color="gray.600"
                maxW="2xl"
                lineHeight="1.6"
              >
                Book time slots and access dedicated robotics development environments with all the tools you need
              </Text>
            </VStack>
            
            <SimpleGrid columns={{ base: 1, md: 2, lg: 4 }} spacing={8} w="full">
              {/* TurtleBot Navigation */}
              <Card
                bg="white"
                border="0"
                borderRadius="12px"
                boxShadow="0 1px 3px rgba(0, 0, 0, 0.1)"
                _hover={{
                  boxShadow: "0 4px 12px rgba(0, 0, 0, 0.15)",
                  transform: "translateY(-2px)",
                }}
                transition="all 0.2s ease"
                overflow="hidden"
              >
                <CardBody p={6}>
                  <VStack spacing={4} align="start">
                    <Box
                      w={12}
                      h={12}
                      bg="blue.500"
                      borderRadius="lg"
                      display="flex"
                      align="center"
                      justify="center"
                      color="white"
                    >
                      <TurtleBotIcon w={6} h={6} />
                    </Box>
                    
                    <Heading fontSize="xl" fontWeight="600" color="gray.900">
                      TurtleBot Navigation
                    </Heading>
                    
                    <Text color="gray.600" lineHeight="1.6">
                      Advanced navigation algorithms, SLAM implementation, and autonomous exploration capabilities for mobile robotics development.
                    </Text>
                    
                    <Button
                      size="sm"
                      variant="ghost"
                      color="blue.500"
                      _hover={{ bg: "blue.50" }}
                      rightIcon={<FaArrowRight />}
                    >
                      Learn More
                    </Button>
                  </VStack>
                </CardBody>
              </Card>

              {/* Robot Arm Manipulation */}
              <Card
                bg="white"
                border="0"
                borderRadius="12px"
                boxShadow="0 1px 3px rgba(0, 0, 0, 0.1)"
                _hover={{
                  boxShadow: "0 4px 12px rgba(0, 0, 0, 0.15)",
                  transform: "translateY(-2px)",
                }}
                transition="all 0.2s ease"
                overflow="hidden"
              >
                <CardBody p={6}>
                  <VStack spacing={4} align="start">
                    <Box
                      w={12}
                      h={12}
                      bg="secondary.500"
                      borderRadius="lg"
                      display="flex"
                      align="center"
                      justify="center"
                      color="white"
                    >
                      <RobotArmIcon w={6} h={6} />
                    </Box>
                    
                    <Heading fontSize="xl" fontWeight="600" color="gray.900">
                      Robot Arm Control
                    </Heading>
                    
                    <Text color="gray.600" lineHeight="1.6">
                      Precise manipulation control, trajectory planning, and object detection for industrial robotics applications.
                    </Text>
                    
                    <Button
                      size="sm"
                      variant="ghost"
                      color="secondary.500"
                      _hover={{ bg: "secondary.50" }}
                      rightIcon={<FaArrowRight />}
                    >
                      Learn More
                    </Button>
                  </VStack>
                </CardBody>
              </Card>

              {/* Development Environment */}
              <Card
                bg="white"
                border="0"
                borderRadius="12px"
                boxShadow="0 1px 3px rgba(0, 0, 0, 0.1)"
                _hover={{
                  boxShadow: "0 4px 12px rgba(0, 0, 0, 0.15)",
                  transform: "translateY(-2px)",
                }}
                transition="all 0.2s ease"
                overflow="hidden"
              >
                <CardBody p={6}>
                  <VStack spacing={4} align="start">
                    <Box
                      w={12}
                      h={12}
                      bg="accent.400"
                      borderRadius="lg"
                      display="flex"
                      align="center"
                      justify="center"
                      color="white"
                    >
                      <Icon as={FaCode} w={6} h={6} />
                    </Box>
                    
                    <Heading fontSize="xl" fontWeight="600" color="gray.900">
                      Code Editor & IDE
                    </Heading>
                    
                    <Text color="gray.600" lineHeight="1.6">
                      Full-featured development environment with syntax highlighting, debugging, and real-time collaboration.
                    </Text>
                    
                    <Button
                      size="sm"
                      variant="ghost"
                      color="accent.400"
                      _hover={{ bg: "accent.50" }}
                      rightIcon={<FaArrowRight />}
                    >
                      Learn More
                    </Button>
                  </VStack>
                </CardBody>
              </Card>

              {/* Simulation Environment */}
              <Card
                bg="white"
                border="0"
                borderRadius="12px"
                boxShadow="0 1px 3px rgba(0, 0, 0, 0.1)"
                _hover={{
                  boxShadow: "0 4px 12px rgba(0, 0, 0, 0.15)",
                  transform: "translateY(-2px)",
                }}
                transition="all 0.2s ease"
                overflow="hidden"
              >
                <CardBody p={6}>
                  <VStack spacing={4} align="start">
                    <Box
                      w={12}
                      h={12}
                      bg="green.500"
                      borderRadius="lg"
                      display="flex"
                      align="center"
                      justify="center"
                      color="white"
                    >
                      <Icon as={FaGlobe} w={6} h={6} />
                    </Box>
                    
                    <Heading fontSize="xl" fontWeight="600" color="gray.900">
                      Gazebo Simulation
                    </Heading>
                    
                    <Text color="gray.600" lineHeight="1.6">
                      Physics-based simulation environment with realistic robot models and environmental interactions.
                    </Text>
                    
                    <Button
                      size="sm"
                      variant="ghost"
                      color="green.500"
                      _hover={{ bg: "green.50" }}
                      rightIcon={<FaArrowRight />}
                    >
                      Learn More
                    </Button>
                  </VStack>
                </CardBody>
              </Card>
            </SimpleGrid>
          </VStack>
        </Container>
      </Box>

      {/* Contact Form */}
      <Box bg="gray.50" py={20} id="contact">
        <Container maxW="4xl">
          <VStack spacing={12} w="full">
            <VStack spacing={6} textAlign="center">
              <Heading
                fontSize={{ base: "3xl", md: "4xl" }}
                fontWeight="700"
                color="gray.900"
                lineHeight="1.2"
              >
                Get in Touch
              </Heading>
              <Text
                fontSize="lg"
                color="gray.600"
                lineHeight="1.6"
              >
                Have questions? Need custom solutions? Our team is here to help.
              </Text>
            </VStack>
            
            <Card
              bg="white"
              border="0"
              borderRadius="16px"
              boxShadow="0 4px 20px rgba(0, 0, 0, 0.1)"
              p={8}
              w="full"
            >
              <VStack spacing={6} w="full">
                <SimpleGrid columns={{ base: 1, md: 2 }} spacing={6} w="full">
                  <FormControl>
                    <FormLabel fontWeight="600" color="gray.700">
                      Name
                    </FormLabel>
                    <Input
                      name="name"
                      value={formData.name}
                      onChange={handleInputChange}
                      placeholder="Your name"
                      size="lg"
                      borderColor="gray.300"
                      _focus={{ borderColor: "blue.500", boxShadow: "0 0 0 1px #3182ce" }}
                    />
                  </FormControl>
                  
                  <FormControl>
                    <FormLabel fontWeight="600" color="gray.700">
                      Email
                    </FormLabel>
                    <Input
                      name="email"
                      type="email"
                      value={formData.email}
                      onChange={handleInputChange}
                      placeholder="your.email@company.com"
                      size="lg"
                      borderColor="gray.300"
                      _focus={{ borderColor: "blue.500", boxShadow: "0 0 0 1px #3182ce" }}
                    />
                  </FormControl>
                </SimpleGrid>
                
                <FormControl>
                  <FormLabel fontWeight="600" color="gray.700">
                    Message
                  </FormLabel>
                  <Textarea
                    name="message"
                    value={formData.message}
                    onChange={handleInputChange}
                    placeholder="Tell us about your robotics development needs..."
                    rows={6}
                    resize="vertical"
                    borderColor="gray.300"
                    _focus={{ borderColor: "blue.500", boxShadow: "0 0 0 1px #3182ce" }}
                  />
                </FormControl>
                
                <Button
                  size="lg"
                  px={8}
                  py={6}
                  fontSize="lg"
                  fontWeight="600"
                  bgGradient="linear(to-r, cyan.400, blue.500)"
                  color="white"
                  boxShadow="0 8px 32px rgba(6, 182, 212, 0.3)"
                  _hover={{
                    bgGradient: "linear(to-r, cyan.300, blue.400)",
                    boxShadow: "0 12px 40px rgba(6, 182, 212, 0.4)",
                    transform: "translateY(-2px)",
                  }}
                  isLoading={isSubmitting}
                  loadingText="Sending..."
                  rightIcon={<FaArrowRight />}
                  onClick={handleSubmitMessage}
                  w="full"
                >
                  Send Message
                </Button>
              </VStack>
            </Card>
          </VStack>
        </Container>
      </Box>

      {/* Footer */}
      <Box
        bg="primary.800"
        color="white"
        py={16}
      >
        <Container maxW="7xl">
          <VStack spacing={12} w="full">
            <SimpleGrid columns={{ base: 1, md: 2, lg: 4 }} spacing={8} w="full">
              {/* Company Info */}
              <VStack align="start" spacing={4}>
                <Heading size="md" color="white" fontWeight="700">
                  RobotConsole
                </Heading>
                <Text color="gray.300" lineHeight="1.6">
                  Professional robotics development platform providing cloud-based console access 
                  for researchers, educators, and industry professionals.
                </Text>
                <HStack spacing={4}>
                  <Link href="#" color="gray.300" _hover={{ color: "white" }}>
                    <Icon as={FaFacebook} w={5} h={5} />
                  </Link>
                  <Link href="#" color="gray.300" _hover={{ color: "white" }}>
                    <Icon as={FaTwitter} w={5} h={5} />
                  </Link>
                  <Link href="#" color="gray.300" _hover={{ color: "white" }}>
                    <Icon as={FaLinkedin} w={5} h={5} />
                  </Link>
                  <Link href="#" color="gray.300" _hover={{ color: "white" }}>
                    <Icon as={FaGithub} w={5} h={5} />
                  </Link>
                </HStack>
              </VStack>

              {/* Platform */}
              <VStack align="start" spacing={4}>
                <Heading size="sm" color="white" fontWeight="600">
                  Platform
                </Heading>
                <VStack align="start" spacing={2}>
                  <Link href="#features" color="gray.300" _hover={{ color: "white" }}>
                    Features
                  </Link>
                  <Link href="#" color="gray.300" _hover={{ color: "white" }}>
                    Pricing
                  </Link>
                  <Link href="#" color="gray.300" _hover={{ color: "white" }}>
                    Documentation
                  </Link>
                  <Link href="#" color="gray.300" _hover={{ color: "white" }}>
                    API Reference
                  </Link>
                </VStack>
              </VStack>

              {/* Support */}
              <VStack align="start" spacing={4}>
                <Heading size="sm" color="white" fontWeight="600">
                  Support
                </Heading>
                <VStack align="start" spacing={2}>
                  <Link href="#" color="gray.300" _hover={{ color: "white" }}>
                    Help Center
                  </Link>
                  <Link href="#contact" color="gray.300" _hover={{ color: "white" }}>
                    Contact Us
                  </Link>
                  <Link href="#" color="gray.300" _hover={{ color: "white" }}>
                    Community
                  </Link>
                  <Link href="#" color="gray.300" _hover={{ color: "white" }}>
                    Status Page
                  </Link>
                </VStack>
              </VStack>

              {/* Contact */}
              <VStack align="start" spacing={4}>
                <Heading size="sm" color="white" fontWeight="600">
                  Contact Info
                </Heading>
                <VStack align="start" spacing={3}>
                  <HStack spacing={3}>
                    <Icon as={FaPhone} color="cyan.400" />
                    <Text color="gray.300" fontSize="sm">
                      +1 (555) 123-4567
                    </Text>
                  </HStack>
                  <HStack spacing={3}>
                    <Icon as={FaEnvelope} color="cyan.400" />
                    <Text color="gray.300" fontSize="sm">
                      contact@robotconsole.com
                    </Text>
                  </HStack>
                  <HStack spacing={3}>
                    <Icon as={FaMapMarkerAlt} color="cyan.400" />
                    <Text color="gray.300" fontSize="sm">
                      San Francisco, CA
                    </Text>
                  </HStack>
                </VStack>
              </VStack>
            </SimpleGrid>
            
            <Divider borderColor="gray.600" />
            
            {/* Copyright */}
            <Flex
              direction={{ base: "column", md: "row" }}
              justify="space-between"
              align="center"
              w="full"
              gap={4}
            >
              <Text color="gray.300" fontSize="sm">
                © 2024 RobotConsole. All rights reserved.
              </Text>
              <HStack spacing={6}>
                <Link href="#" color="gray.300" fontSize="sm" _hover={{ color: "white" }}>
                  Privacy Policy
                </Link>
                <Link href="#" color="gray.300" fontSize="sm" _hover={{ color: "white" }}>
                  Terms of Service
                </Link>
                <Link href="#" color="gray.300" fontSize="sm" _hover={{ color: "white" }}>
                  Cookie Policy
                </Link>
              </HStack>
            </Flex>
          </VStack>
        </Container>
      </Box>
    </Box>
  );
};

export default LandingPage;
