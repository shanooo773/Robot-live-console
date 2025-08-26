import {
  Box,
  VStack,
  HStack,
  Text,
  Button,
  Container,
  Card,
  CardBody,
  CardHeader,
  SimpleGrid,
  Badge,
  Avatar,
  Divider,
  useToast,
  Select,
  Input,
  FormControl,
  FormLabel,
} from "@chakra-ui/react";
import { useState, useEffect } from "react";
import { motion } from "framer-motion";

// Motion components
const MotionBox = motion(Box);
const MotionCard = motion(Card);
const MotionVStack = motion(VStack);
const MotionHStack = motion(HStack);

// Dummy data for available time slots
const generateTimeSlots = () => {
  const slots = [];
  const today = new Date();
  
  for (let day = 0; day < 7; day++) {
    const date = new Date(today);
    date.setDate(today.getDate() + day);
    
    // Generate slots from 9 AM to 9 PM
    for (let hour = 9; hour <= 21; hour++) {
      const startTime = new Date(date);
      startTime.setHours(hour, 0, 0, 0);
      
      const endTime = new Date(startTime);
      endTime.setHours(hour + 1, 0, 0, 0);
      
      // Randomly mark some slots as taken to simulate real usage
      const isTaken = Math.random() < 0.3;
      
      slots.push({
        id: `slot_${day}_${hour}`,
        date: date.toISOString().split('T')[0],
        startTime: startTime.toLocaleTimeString('en-US', { 
          hour: 'numeric', 
          minute: '2-digit',
          hour12: true 
        }),
        endTime: endTime.toLocaleTimeString('en-US', { 
          hour: 'numeric', 
          minute: '2-digit',
          hour12: true 
        }),
        available: !isTaken,
        robotType: ["turtlebot", "arm", "hand"][Math.floor(Math.random() * 3)],
        bookedBy: isTaken ? "Another User" : null,
      });
    }
  }
  
  return slots;
};

const robotNames = {
  turtlebot: { name: "TurtleBot3", emoji: "🤖" },
  arm: { name: "Robot Arm", emoji: "🦾" },
  hand: { name: "Robot Hand", emoji: "🤲" },
};

const BookingPage = ({ user, onBooking, onLogout }) => {
  const [timeSlots, setTimeSlots] = useState([]);
  const [selectedDate, setSelectedDate] = useState("");
  const [selectedRobot, setSelectedRobot] = useState("");
  const [isLoading, setIsLoading] = useState(false);
  const toast = useToast();

  useEffect(() => {
    // Generate dummy time slots on component mount
    const slots = generateTimeSlots();
    setTimeSlots(slots);
    
    // Set default date to today
    const today = new Date().toISOString().split('T')[0];
    setSelectedDate(today);
  }, []);

  const filteredSlots = timeSlots.filter(slot => {
    let matches = true;
    if (selectedDate && slot.date !== selectedDate) matches = false;
    if (selectedRobot && slot.robotType !== selectedRobot) matches = false;
    return matches;
  });

  const availableSlots = filteredSlots.filter(slot => slot.available);
  const bookedSlots = filteredSlots.filter(slot => !slot.available);

  const handleBookSlot = async (slot) => {
    setIsLoading(true);
    
    // Simulate booking API call
    setTimeout(() => {
      const bookedSlot = {
        ...slot,
        available: false,
        bookedBy: user.name,
        bookingTime: new Date().toISOString(),
      };
      
      // Update the slot status
      setTimeSlots(prev => prev.map(s => 
        s.id === slot.id ? bookedSlot : s
      ));
      
      onBooking(bookedSlot);
      
      toast({
        title: "Booking confirmed!",
        description: `Your session is booked for ${slot.date} at ${slot.startTime}`,
        status: "success",
        duration: 5000,
        isClosable: true,
      });
      
      setIsLoading(false);
    }, 1000);
  };

  const getDateOptions = () => {
    const today = new Date();
    const options = [];
    
    for (let i = 0; i < 7; i++) {
      const date = new Date(today);
      date.setDate(today.getDate() + i);
      const dateStr = date.toISOString().split('T')[0];
      const label = i === 0 ? 'Today' : 
                   i === 1 ? 'Tomorrow' : 
                   date.toLocaleDateString('en-US', { weekday: 'long', month: 'short', day: 'numeric' });
      options.push({ value: dateStr, label });
    }
    
    return options;
  };

  return (
    <Container maxW="7xl" py={8}>
      <MotionVStack 
        spacing={8}
        initial={{ opacity: 0 }}
        animate={{ opacity: 1 }}
        transition={{ duration: 0.6 }}
      >
        {/* Header */}
        <MotionHStack 
          w="full" 
          justify="space-between"
          initial={{ opacity: 0, y: -20 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.6, delay: 0.1 }}
        >
          <VStack align="start" spacing={2}>
            <Text 
              fontSize="4xl" 
              fontWeight="bold" 
              bgGradient="linear(to-r, #667eea, #764ba2)"
              bgClip="text"
            >
              Book Your Robot Session
            </Text>
            <HStack spacing={3}>
              <Avatar size="sm" name={user.name} />
              <VStack align="start" spacing={0}>
                <Text color="white" fontWeight="500">Welcome, {user.name}</Text>
                <Text color="gray.400" fontSize="sm">{user.email}</Text>
              </VStack>
            </HStack>
          </VStack>
          <Button
            variant="glassmorphism"
            onClick={onLogout}
            _hover={{
              transform: "translateY(-2px)",
              background: "rgba(255, 255, 255, 0.2)",
            }}
          >
            Logout
          </Button>
        </MotionHStack>

        {/* Filters */}
        <MotionCard 
          w="full" 
          variant="glassmorphism"
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.6, delay: 0.2 }}
        >
          <CardHeader pb={2}>
            <Text fontSize="xl" fontWeight="bold" color="white">
              Filter Available Sessions
            </Text>
          </CardHeader>
          <CardBody>
            <SimpleGrid columns={{ base: 1, md: 2 }} spacing={6}>
              <FormControl>
                <FormLabel color="gray.300" fontSize="sm" fontWeight="500">
                  Date
                </FormLabel>
                <Select
                  value={selectedDate}
                  onChange={(e) => setSelectedDate(e.target.value)}
                  variant="glassmorphism"
                  size="lg"
                  color="white"
                  _focus={{
                    borderColor: "blue.400",
                    boxShadow: "0 0 0 1px rgba(66, 153, 225, 0.6)",
                  }}
                >
                  <option value="" style={{background: '#2D3748', color: 'white'}}>All dates</option>
                  {getDateOptions().map(option => (
                    <option key={option.value} value={option.value} style={{background: '#2D3748', color: 'white'}}>
                      {option.label}
                    </option>
                  ))}
                </Select>
              </FormControl>

              <FormControl>
                <FormLabel color="gray.300" fontSize="sm" fontWeight="500">
                  Robot Type
                </FormLabel>
                <Select
                  value={selectedRobot}
                  onChange={(e) => setSelectedRobot(e.target.value)}
                  variant="glassmorphism"
                  size="lg"
                  color="white"
                  _focus={{
                    borderColor: "blue.400",
                    boxShadow: "0 0 0 1px rgba(66, 153, 225, 0.6)",
                  }}
                >
                  <option value="" style={{background: '#2D3748', color: 'white'}}>All robots</option>
                  <option value="turtlebot" style={{background: '#2D3748', color: 'white'}}>🤖 TurtleBot3</option>
                  <option value="arm" style={{background: '#2D3748', color: 'white'}}>🦾 Robot Arm</option>
                  <option value="hand" style={{background: '#2D3748', color: 'white'}}>🤲 Robot Hand</option>
                </Select>
              </FormControl>
            </SimpleGrid>
          </CardBody>
        </MotionCard>

        {/* Available Slots */}
        <MotionVStack 
          w="full" 
          spacing={6}
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.6, delay: 0.3 }}
        >
          <MotionHStack 
            w="full" 
            justify="space-between"
            initial={{ opacity: 0, x: -20 }}
            animate={{ opacity: 1, x: 0 }}
            transition={{ duration: 0.5, delay: 0.4 }}
          >
            <Text fontSize="2xl" fontWeight="bold" color="white">
              Available Sessions ({availableSlots.length})
            </Text>
            <Badge 
              variant="gradient" 
              fontSize="sm" 
              px={4} 
              py={2}
              borderRadius="full"
            >
              {availableSlots.length} slots free
            </Badge>
          </MotionHStack>

          {availableSlots.length === 0 ? (
            <MotionCard 
              w="full" 
              variant="glassmorphism"
              initial={{ opacity: 0, scale: 0.95 }}
              animate={{ opacity: 1, scale: 1 }}
              transition={{ duration: 0.5, delay: 0.5 }}
            >
              <CardBody textAlign="center" py={16}>
                <MotionBox
                  initial={{ scale: 0 }}
                  animate={{ scale: 1 }}
                  transition={{ duration: 0.5, delay: 0.7, type: "spring" }}
                >
                  <Text fontSize="4xl" mb={4}>🔍</Text>
                </MotionBox>
                <Text fontSize="xl" color="gray.300" mb={3} fontWeight="500">
                  No available slots found
                </Text>
                <Text color="gray.400">
                  Try adjusting your filters or check back later
                </Text>
              </CardBody>
            </MotionCard>
          ) : (
            <SimpleGrid columns={{ base: 1, md: 2, lg: 3 }} spacing={6} w="full">
              {availableSlots.map((slot, index) => (
                <MotionCard
                  key={slot.id}
                  variant="glassmorphism"
                  cursor="pointer"
                  initial={{ opacity: 0, y: 50, scale: 0.9 }}
                  animate={{ opacity: 1, y: 0, scale: 1 }}
                  transition={{ 
                    duration: 0.5, 
                    delay: 0.5 + (index * 0.1),
                    type: "spring",
                    stiffness: 100
                  }}
                  whileHover={{ 
                    y: -8,
                    scale: 1.02,
                    transition: { duration: 0.2 }
                  }}
                  whileTap={{ scale: 0.98 }}
                >
                  <CardBody p={6}>
                    <VStack spacing={4} align="start">
                      <HStack justify="space-between" w="full">
                        <Badge 
                          variant="glassmorphism" 
                          colorScheme="green"
                          fontSize="xs"
                          px={3}
                          py={1}
                        >
                          Available
                        </Badge>
                        <HStack spacing={2}>
                          <MotionBox
                            fontSize="2xl"
                            animate={{
                              rotate: [0, 10, -10, 0],
                              transition: {
                                duration: 2,
                                repeat: Infinity,
                                delay: index * 0.2
                              }
                            }}
                          >
                            {robotNames[slot.robotType].emoji}
                          </MotionBox>
                          <VStack align="start" spacing={0}>
                            <Text fontSize="sm" color="white" fontWeight="500">
                              {robotNames[slot.robotType].name}
                            </Text>
                          </VStack>
                        </HStack>
                      </HStack>
                      
                      <VStack align="start" spacing={2} w="full">
                        <Text color="white" fontWeight="bold" fontSize="lg">
                          {new Date(slot.date).toLocaleDateString('en-US', { 
                            weekday: 'long', 
                            month: 'short', 
                            day: 'numeric' 
                          })}
                        </Text>
                        <Text 
                          color="gray.300" 
                          fontSize="md"
                          fontWeight="500"
                        >
                          {slot.startTime} - {slot.endTime}
                        </Text>
                      </VStack>

                      <Button
                        variant="gradient"
                        w="full"
                        onClick={() => handleBookSlot(slot)}
                        isLoading={isLoading}
                        loadingText="Booking..."
                        size="lg"
                        fontWeight="600"
                        _hover={{
                          transform: "translateY(-2px)",
                          boxShadow: "0 10px 25px rgba(72, 187, 120, 0.3)",
                        }}
                      >
                        Book This Session ✨
                      </Button>
                    </VStack>
                  </CardBody>
                </MotionCard>
              ))}
            </SimpleGrid>
          )}
        </MotionVStack>

        {/* Booked Slots (for reference) */}
        {bookedSlots.length > 0 && (
          <MotionVStack 
            w="full" 
            spacing={6}
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.6, delay: 0.8 }}
          >
            <Divider borderColor="whiteAlpha.200" />
            
            <MotionHStack 
              w="full" 
              justify="space-between"
              initial={{ opacity: 0, x: -20 }}
              animate={{ opacity: 1, x: 0 }}
              transition={{ duration: 0.5, delay: 0.9 }}
            >
              <Text fontSize="2xl" fontWeight="bold" color="white">
                Unavailable Sessions
              </Text>
              <Badge 
                colorScheme="red" 
                fontSize="sm"
                px={4} 
                py={2}
                borderRadius="full"
                variant="outline"
              >
                {bookedSlots.length} slots taken
              </Badge>
            </MotionHStack>

            <SimpleGrid columns={{ base: 1, md: 2, lg: 3 }} spacing={6} w="full">
              {bookedSlots.slice(0, 6).map((slot, index) => (
                <MotionCard
                  key={slot.id}
                  bg="rgba(255, 255, 255, 0.02)"
                  backdropFilter="blur(10px)"
                  border="1px solid rgba(255, 255, 255, 0.05)"
                  borderRadius="2xl"
                  opacity={0.6}
                  initial={{ opacity: 0, y: 30 }}
                  animate={{ opacity: 0.6, y: 0 }}
                  transition={{ 
                    duration: 0.5, 
                    delay: 1 + (index * 0.1)
                  }}
                >
                  <CardBody p={6}>
                    <VStack spacing={4} align="start">
                      <HStack justify="space-between" w="full">
                        <Badge 
                          colorScheme="red" 
                          variant="solid"
                          fontSize="xs"
                          px={3}
                          py={1}
                        >
                          Taken
                        </Badge>
                        <HStack spacing={2}>
                          <Text fontSize="xl" opacity={0.7}>
                            {robotNames[slot.robotType].emoji}
                          </Text>
                          <VStack align="start" spacing={0}>
                            <Text fontSize="sm" color="gray.400">
                              {robotNames[slot.robotType].name}
                            </Text>
                          </VStack>
                        </HStack>
                      </HStack>
                      
                      <VStack align="start" spacing={2} w="full">
                        <Text color="gray.300" fontWeight="500">
                          {new Date(slot.date).toLocaleDateString('en-US', { 
                            weekday: 'long', 
                            month: 'short', 
                            day: 'numeric' 
                          })}
                        </Text>
                        <Text color="gray.400" fontSize="sm">
                          {slot.startTime} - {slot.endTime}
                        </Text>
                      </VStack>

                      <Text fontSize="sm" color="gray.500" fontStyle="italic">
                        Booked by Another User
                      </Text>
                    </VStack>
                  </CardBody>
                </MotionCard>
              ))}
            </SimpleGrid>
          </MotionVStack>
        )}
      </MotionVStack>
    </Container>
  );
};

export default BookingPage;