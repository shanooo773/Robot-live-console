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
  InputGroup,
  InputRightElement,
  IconButton,
} from "@chakra-ui/react";
import { motion } from "framer-motion";
import { useState } from "react";

const MotionBox = motion(Box);
const MotionCard = motion(Card);
const MotionText = motion(Text);
const MotionButton = motion(Button);
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
        spacing={12}
        initial={{ opacity: 0, y: 50 }}
        animate={{ opacity: 1, y: 0 }}
        transition={{ duration: 0.8 }}
      >
        {/* Header */}
        <MotionBox
          initial={{ opacity: 0, scale: 0.8 }}
          animate={{ opacity: 1, scale: 1 }}
          transition={{ delay: 0.2, duration: 0.8 }}
        >
          <VStack spacing={6} textAlign="center">
            <MotionText 
              fontSize="6xl"
              animate={{ 
                y: [0, -10, 0],
                rotate: [0, 5, -5, 0]
              }}
              transition={{ 
                duration: 3, 
                repeat: Infinity, 
                ease: "easeInOut" 
              }}
              filter="drop-shadow(0 0 20px rgba(56, 189, 248, 0.3))"
            >
              🤖
            </MotionText>
            <MotionText 
              fontSize={{ base: "2xl", md: "3xl" }}
              fontWeight="bold" 
              bgGradient="linear(to-r, brand.400, accent.400)"
              bgClip="text"
              initial={{ opacity: 0 }}
              animate={{ opacity: 1 }}
              transition={{ delay: 0.4, duration: 0.8 }}
            >
              Welcome Back
            </MotionText>
            <MotionText 
              color="gray.300" 
              fontSize="lg"
              initial={{ opacity: 0 }}
              animate={{ opacity: 1 }}
              transition={{ delay: 0.6, duration: 0.8 }}
            >
              Sign in to book your robot programming session
            </MotionText>
          </VStack>
        </MotionBox>

        {/* Auth Card */}
        <MotionCard 
          w="full" 
          variant="glass"
          initial={{ opacity: 0, y: 30 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ delay: 0.8, duration: 0.8 }}
        >
          <CardBody p={8}>
            <Tabs variant="enclosed" colorScheme="brand">
              <TabList mb={6} borderRadius="xl" bg="rgba(255, 255, 255, 0.05)">
                <Tab 
                  color="gray.300" 
                  _selected={{ 
                    color: "white", 
                    bg: "linear-gradient(135deg, brand.500 0%, brand.600 100%)",
                    borderRadius: "lg"
                  }}
                  borderRadius="lg"
                  fontWeight="600"
                >
                  Sign In
                </Tab>
                <Tab 
                  color="gray.300" 
                  _selected={{ 
                    color: "white", 
                    bg: "linear-gradient(135deg, brand.500 0%, brand.600 100%)",
                    borderRadius: "lg"
                  }}
                  borderRadius="lg"
                  fontWeight="600"
                >
                  Sign Up
                </Tab>
              </TabList>

              <TabPanels>
                {/* Login Tab */}
                <TabPanel px={0}>
                  <form onSubmit={handleLogin}>
                    <VStack spacing={4}>
                      <FormControl>
                        <FormLabel color="gray.300" fontWeight="600">Email</FormLabel>
                        <Input
                          type="email"
                          placeholder="Enter your email"
                          value={loginData.email}
                          onChange={(e) => setLoginData({...loginData, email: e.target.value})}
                          variant="glass"
                          size="lg"
                          _placeholder={{ color: "gray.500" }}
                        />
                      </FormControl>

                      <FormControl>
                        <FormLabel color="gray.300" fontWeight="600">Password</FormLabel>
                        <Input
                          type="password"
                          placeholder="Enter your password"
                          value={loginData.password}
                          onChange={(e) => setLoginData({...loginData, password: e.target.value})}
                          variant="glass"
                          size="lg"
                          _placeholder={{ color: "gray.500" }}
                        />
                      </FormControl>

                      <MotionButton
                        type="submit"
                        variant="gradient"
                        w="full"
                        size="lg"
                        isLoading={isLoading}
                        loadingText="Signing in..."
                        whileHover={{ scale: 1.02 }}
                        whileTap={{ scale: 0.98 }}
                      >
                        Sign In
                      </MotionButton>
                    </VStack>
                  </form>
                </TabPanel>

                {/* Register Tab */}
                <TabPanel px={0}>
                  <form onSubmit={handleRegister}>
                    <VStack spacing={6}>
                      <FormControl>
                        <FormLabel color="gray.300" fontWeight="600">Full Name</FormLabel>
                        <Input
                          type="text"
                          placeholder="Enter your full name"
                          value={registerData.name}
                          onChange={(e) => setRegisterData({...registerData, name: e.target.value})}
                          variant="glass"
                          size="lg"
                          _placeholder={{ color: "gray.500" }}
                        />
                      </FormControl>

                      <FormControl>
                        <FormLabel color="gray.300" fontWeight="600">Email</FormLabel>
                        <Input
                          type="email"
                          placeholder="Enter your email"
                          value={registerData.email}
                          onChange={(e) => setRegisterData({...registerData, email: e.target.value})}
                          variant="glass"
                          size="lg"
                          _placeholder={{ color: "gray.500" }}
                        />
                      </FormControl>

                      <FormControl>
                        <FormLabel color="gray.300" fontWeight="600">Password</FormLabel>
                        <Input
                          type="password"
                          placeholder="Create a password"
                          value={registerData.password}
                          onChange={(e) => setRegisterData({...registerData, password: e.target.value})}
                          variant="glass"
                          size="lg"
                          _placeholder={{ color: "gray.500" }}
                        />
                      </FormControl>

                      <FormControl>
                        <FormLabel color="gray.300" fontWeight="600">Confirm Password</FormLabel>
                        <Input
                          type="password"
                          placeholder="Confirm your password"
                          value={registerData.confirmPassword}
                          onChange={(e) => setRegisterData({...registerData, confirmPassword: e.target.value})}
                          variant="glass"
                          size="lg"
                          _placeholder={{ color: "gray.500" }}
                        />
                      </FormControl>

                      <MotionButton
                        type="submit"
                        variant="gradient"
                        w="full"
                        size="lg"
                        isLoading={isLoading}
                        loadingText="Creating account..."
                        whileHover={{ scale: 1.02 }}
                        whileTap={{ scale: 0.98 }}
                      >
                        Create Account
                      </MotionButton>
                    </VStack>
                  </form>
                </TabPanel>
              </TabPanels>
            </Tabs>
          </CardBody>
        </MotionCard>

        {/* Back Button */}
        <MotionButton
          variant="glass"
          onClick={onBack}
          initial={{ opacity: 0 }}
          animate={{ opacity: 1 }}
          transition={{ delay: 1.0, duration: 0.8 }}
          whileHover={{ scale: 1.05 }}
          whileTap={{ scale: 0.95 }}
        >
          ← Back to Home
        </MotionButton>
      </MotionVStack>
    </Container>
  );
};

export default AuthPage;