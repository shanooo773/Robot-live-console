import {
  Box,
  VStack,
  HStack,
  Text,
  Button,
  Input,
  FormControl,
  FormLabel,
  Container,
  Card,
  CardBody,
  CardHeader,
  Tabs,
  TabList,
  TabPanels,
  Tab,
  TabPanel,
  useToast,
  Alert,
  AlertIcon,
} from "@chakra-ui/react";
import { useState } from "react";
import { motion } from "framer-motion";

// Motion components
const MotionBox = motion(Box);
const MotionCard = motion(Card);
const MotionVStack = motion(VStack);

const AuthPage = ({ onAuth, onBack }) => {
  const [isLoading, setIsLoading] = useState(false);
  const [loginData, setLoginData] = useState({ email: "", password: "" });
  const [registerData, setRegisterData] = useState({
    name: "",
    email: "",
    password: "",
    confirmPassword: ""
  });
  const toast = useToast();

  const handleLogin = async (e) => {
    e.preventDefault();
    setIsLoading(true);

    // Simulate API call with dummy validation
    setTimeout(() => {
      if (loginData.email && loginData.password) {
        const userData = {
          id: "user_123",
          name: loginData.email.split("@")[0],
          email: loginData.email,
        };
        onAuth(userData);
        toast({
          title: "Login successful",
          status: "success",
          duration: 3000,
          isClosable: true,
        });
      } else {
        toast({
          title: "Login failed",
          description: "Please enter both email and password",
          status: "error",
          duration: 3000,
          isClosable: true,
        });
      }
      setIsLoading(false);
    }, 1000);
  };

  const handleRegister = async (e) => {
    e.preventDefault();
    setIsLoading(true);

    // Simulate API call with dummy validation
    setTimeout(() => {
      if (registerData.name && registerData.email && registerData.password) {
        if (registerData.password !== registerData.confirmPassword) {
          toast({
            title: "Registration failed",
            description: "Passwords do not match",
            status: "error",
            duration: 3000,
            isClosable: true,
          });
          setIsLoading(false);
          return;
        }

        const userData = {
          id: "user_" + Date.now(),
          name: registerData.name,
          email: registerData.email,
        };
        onAuth(userData);
        toast({
          title: "Registration successful",
          description: "Welcome to Robot Programming Console!",
          status: "success",
          duration: 3000,
          isClosable: true,
        });
      } else {
        toast({
          title: "Registration failed",
          description: "Please fill in all fields",
          status: "error",
          duration: 3000,
          isClosable: true,
        });
      }
      setIsLoading(false);
    }, 1000);
  };

  return (
    <Container maxW="md" py={20}>
      <MotionVStack 
        spacing={8}
        initial={{ opacity: 0, y: 50 }}
        animate={{ opacity: 1, y: 0 }}
        transition={{ duration: 0.8, ease: "easeOut" }}
      >
        {/* Header */}
        <MotionVStack 
          spacing={4} 
          textAlign="center"
          initial={{ opacity: 0, scale: 0.8 }}
          animate={{ opacity: 1, scale: 1 }}
          transition={{ duration: 0.6, delay: 0.2 }}
        >
          <MotionBox
            fontSize="5xl"
            initial={{ rotate: -180, scale: 0 }}
            animate={{ rotate: 0, scale: 1 }}
            transition={{ duration: 0.8, delay: 0.4, type: "spring", stiffness: 200 }}
          >
            🤖
          </MotionBox>
          <MotionBox
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.6, delay: 0.6 }}
          >
            <Text 
              fontSize="3xl" 
              fontWeight="bold" 
              bgGradient="linear(to-r, #667eea, #764ba2)"
              bgClip="text"
            >
              Welcome Back
            </Text>
          </MotionBox>
          <MotionBox
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.6, delay: 0.8 }}
          >
            <Text color="gray.300" fontSize="lg">
              Sign in to book your robot programming session
            </Text>
          </MotionBox>
        </MotionVStack>

        {/* Auth Card */}
        <MotionCard 
          w="full" 
          variant="glassmorphism"
          initial={{ opacity: 0, y: 30 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.6, delay: 1 }}
        >
          <CardBody p={8}>
            <Tabs variant="soft-rounded" colorScheme="blue">
              <TabList mb={6} bg="rgba(255, 255, 255, 0.05)" p={1} borderRadius="xl">
                <Tab 
                  color="gray.300" 
                  _selected={{ 
                    color: "white", 
                    bg: "gradient.blue",
                    transform: "scale(1.05)"
                  }}
                  borderRadius="lg"
                  transition="all 0.2s"
                >
                  Sign In
                </Tab>
                <Tab 
                  color="gray.300" 
                  _selected={{ 
                    color: "white", 
                    bg: "gradient.blue",
                    transform: "scale(1.05)"
                  }}
                  borderRadius="lg"
                  transition="all 0.2s"
                >
                  Sign Up
                </Tab>
              </TabList>

              <TabPanels>
                {/* Login Tab */}
                <TabPanel px={0}>
                  <MotionBox
                    initial={{ opacity: 0, x: -20 }}
                    animate={{ opacity: 1, x: 0 }}
                    transition={{ duration: 0.4 }}
                  >
                    <form onSubmit={handleLogin}>
                      <VStack spacing={6}>
                        <FormControl>
                          <FormLabel color="gray.300" fontSize="sm" fontWeight="500">
                            Email Address
                          </FormLabel>
                          <Input
                            type="email"
                            placeholder="Enter your email"
                            value={loginData.email}
                            onChange={(e) => setLoginData({...loginData, email: e.target.value})}
                            variant="glassmorphism"
                            size="lg"
                            fontSize="md"
                          />
                        </FormControl>

                        <FormControl>
                          <FormLabel color="gray.300" fontSize="sm" fontWeight="500">
                            Password
                          </FormLabel>
                          <Input
                            type="password"
                            placeholder="Enter your password"
                            value={loginData.password}
                            onChange={(e) => setLoginData({...loginData, password: e.target.value})}
                            variant="glassmorphism"
                            size="lg"
                            fontSize="md"
                          />
                        </FormControl>

                        <Button
                          type="submit"
                          variant="gradient"
                          size="lg"
                          w="full"
                          isLoading={isLoading}
                          loadingText="Signing in..."
                          fontSize="md"
                          fontWeight="600"
                        >
                          Sign In
                        </Button>
                      </VStack>
                    </form>
                  </MotionBox>
                </TabPanel>

                {/* Register Tab */}
                <TabPanel px={0}>
                  <MotionBox
                    initial={{ opacity: 0, x: 20 }}
                    animate={{ opacity: 1, x: 0 }}
                    transition={{ duration: 0.4 }}
                  >
                    <form onSubmit={handleRegister}>
                      <VStack spacing={6}>
                        <FormControl>
                          <FormLabel color="gray.300" fontSize="sm" fontWeight="500">
                            Full Name
                          </FormLabel>
                          <Input
                            type="text"
                            placeholder="Enter your full name"
                            value={registerData.name}
                            onChange={(e) => setRegisterData({...registerData, name: e.target.value})}
                            variant="glassmorphism"
                            size="lg"
                            fontSize="md"
                          />
                        </FormControl>

                        <FormControl>
                          <FormLabel color="gray.300" fontSize="sm" fontWeight="500">
                            Email Address
                          </FormLabel>
                          <Input
                            type="email"
                            placeholder="Enter your email"
                            value={registerData.email}
                            onChange={(e) => setRegisterData({...registerData, email: e.target.value})}
                            variant="glassmorphism"
                            size="lg"
                            fontSize="md"
                          />
                        </FormControl>

                        <FormControl>
                          <FormLabel color="gray.300" fontSize="sm" fontWeight="500">
                            Password
                          </FormLabel>
                          <Input
                            type="password"
                            placeholder="Choose a password"
                            value={registerData.password}
                            onChange={(e) => setRegisterData({...registerData, password: e.target.value})}
                            variant="glassmorphism"
                            size="lg"
                            fontSize="md"
                          />
                        </FormControl>

                        <FormControl>
                          <FormLabel color="gray.300" fontSize="sm" fontWeight="500">
                            Confirm Password
                          </FormLabel>
                          <Input
                            type="password"
                            placeholder="Confirm your password"
                            value={registerData.confirmPassword}
                            onChange={(e) => setRegisterData({...registerData, confirmPassword: e.target.value})}
                            variant="glassmorphism"
                            size="lg"
                            fontSize="md"
                          />
                        </FormControl>

                        <Button
                          type="submit"
                          variant="gradient"
                          size="lg"
                          w="full"
                          isLoading={isLoading}
                          loadingText="Creating account..."
                          fontSize="md"
                          fontWeight="600"
                        >
                          Create Account
                        </Button>
                      </VStack>
                    </form>
                  </MotionBox>
                </TabPanel>
              </TabPanels>
            </Tabs>
          </CardBody>
        </MotionCard>

        {/* Demo Notice */}
        <MotionBox
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.6, delay: 1.2 }}
        >
          <Alert 
            status="info" 
            borderRadius="xl"
            bg="rgba(66, 153, 225, 0.1)"
            border="1px solid rgba(66, 153, 225, 0.3)"
          >
            <AlertIcon />
            <Text color="gray.300">
              This is a demo system. Use any email/password to test the functionality.
            </Text>
          </Alert>
        </MotionBox>

        {/* Back Button */}
        <MotionBox
          initial={{ opacity: 0 }}
          animate={{ opacity: 1 }}
          transition={{ duration: 0.6, delay: 1.4 }}
        >
          <Button
            variant="ghost"
            onClick={onBack}
            color="gray.400"
            _hover={{
              color: "white",
              transform: "translateY(-2px)",
            }}
          >
            ← Back to Home
          </Button>
        </MotionBox>
      </MotionVStack>
    </Container>
  );
};

export default AuthPage;