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
  Flex,
  Spacer,
  Icon,
} from "@chakra-ui/react";
import { motion } from "framer-motion";
import { useState, useEffect } from "react";

const MotionBox = motion(Box);
const MotionCard = motion(Card);
const MotionText = motion(Text);
const MotionButton = motion(Button);
const MotionVStack = motion(VStack);
const MotionSimpleGrid = motion(SimpleGrid);

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
        spacing={10}
        initial={{ opacity: 0, y: 50 }}
        animate={{ opacity: 1, y: 0 }}
        transition={{ duration: 0.8 }}
      >
        {/* Header */}
        <MotionBox 
          w="full"
          initial={{ opacity: 0, y: 30 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ delay: 0.2, duration: 0.8 }}
        >
          <Flex align="center" justify="space-between" w="full">
            <VStack align="start" spacing={3}>
              <MotionText 
                fontSize={{ base: "2xl", md: "4xl" }} 
                fontWeight="bold" 
                bgGradient="linear(to-r, brand.400, accent.400)"
                bgClip="text"
              >
                Book Your Robot Session
              </MotionText>
              <HStack spacing={3}>
                <Avatar 
                  size="sm" 
                  name={user.name}
                  border="2px solid"
                  borderColor="brand.400"
                />
                <VStack align="start" spacing={0}>
                  <Text color="white" fontWeight="600">Welcome back, {user.name}</Text>
                  <Text color="gray.400" fontSize="sm">{user.email}</Text>
                </VStack>
              </HStack>
            </VStack>
            <MotionButton
              variant="glass"
              onClick={onLogout}
              whileHover={{ scale: 1.05 }}
              whileTap={{ scale: 0.95 }}
            >
              Logout
            </MotionButton>
          </Flex>
        </MotionBox>

        {/* Filters */}
        <MotionCard 
          w="full" 
          variant="glass"
          initial={{ opacity: 0, y: 30 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ delay: 0.4, duration: 0.8 }}
        >
          <CardHeader>
            <Flex align="center" gap={3}>
              <Box fontSize="xl">🔍</Box>
              <Text fontSize="xl" fontWeight="bold" color="white">
                Filter Available Sessions
              </Text>
            </Flex>
          </CardHeader>
          <CardBody>
            <SimpleGrid columns={{ base: 1, md: 2 }} spacing={6}>
              <FormControl>
                <FormLabel color="gray.300" fontWeight="600">Date</FormLabel>
                <Select
                  value={selectedDate}
                  onChange={(e) => setSelectedDate(e.target.value)}
                  variant="glass"
                  size="lg"
                >
                  {getDateOptions().map(option => (
                    <option key={option.value} value={option.value} style={{ backgroundColor: '#1e293b', color: 'white' }}>
                      {option.label}
                    </option>
                  ))}
                </Select>
              </FormControl>
              
              <FormControl>
                <FormLabel color="gray.300" fontWeight="600">Robot Type</FormLabel>
                <Select
                  value={selectedRobot}
                  onChange={(e) => setSelectedRobot(e.target.value)}
                  variant="glass"
                  size="lg"
                >
                  <option value="" style={{ backgroundColor: '#1e293b', color: 'white' }}>All robots</option>
                  <option value="turtlebot" style={{ backgroundColor: '#1e293b', color: 'white' }}>🤖 TurtleBot3</option>
                  <option value="arm" style={{ backgroundColor: '#1e293b', color: 'white' }}>🦾 Robot Arm</option>
                  <option value="hand" style={{ backgroundColor: '#1e293b', color: 'white' }}>🤲 Robot Hand</option>
                </Select>
              </FormControl>
            </SimpleGrid>
          </CardBody>
        </MotionCard>

        {/* Available Slots */}
        <MotionBox 
          w="full"
          initial={{ opacity: 0, y: 30 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ delay: 0.6, duration: 0.8 }}
        >
          <VStack w="full" spacing={8}>
            <Flex w="full" justify="space-between" align="center">
              <MotionText 
                fontSize={{ base: "xl", md: "2xl" }} 
                fontWeight="bold" 
                color="white"
              >
                Available Sessions ({availableSlots.length})
              </MotionText>
              <Badge 
                variant="gradient" 
                px={4} 
                py={2}
                fontSize="md"
                borderRadius="full"
              >
                {availableSlots.length} slots free
              </Badge>
            </Flex>

            {availableSlots.length === 0 ? (
              <MotionCard 
                w="full" 
                variant="glass"
                whileHover={{ scale: 1.02 }}
                transition={{ type: "spring", stiffness: 300 }}
              >
                <CardBody textAlign="center" py={16}>
                  <VStack spacing={4}>
                    <Text fontSize="6xl" opacity="0.5">😔</Text>
                    <Text fontSize="xl" color="gray.400" fontWeight="600">
                      No available slots found
                    </Text>
                    <Text color="gray.500">
                      Try adjusting your filters or check back later
                    </Text>
                  </VStack>
                </CardBody>
              </MotionCard>
            ) : (
              <MotionSimpleGrid 
                columns={{ base: 1, md: 2, lg: 3 }} 
                spacing={6} 
                w="full"
                initial={{ opacity: 0 }}
                animate={{ opacity: 1 }}
                transition={{ delay: 0.8, duration: 0.8 }}
              >
                {availableSlots.map((slot, index) => (
                  <MotionCard
                    key={slot.id}
                    variant="glass"
                    cursor="pointer"
                    whileHover={{ 
                      y: -8, 
                      scale: 1.02,
                      boxShadow: "0 20px 40px rgba(56, 189, 248, 0.2)"
                    }}
                    whileTap={{ scale: 0.98 }}
                    transition={{ 
                      type: "spring", 
                      stiffness: 300,
                      delay: index * 0.1 
                    }}
                    initial={{ opacity: 0, y: 30 }}
                    animate={{ opacity: 1, y: 0 }}
                    onClick={() => handleBookSlot(slot)}
                  >
                    <CardBody p={6}>
                      <VStack spacing={4} align="start">
                        <Flex justify="space-between" w="full" align="center">
                          <Badge variant="glass" colorScheme="green" px={3} py={1}>
                            ✅ Available
                          </Badge>
                          <HStack>
                            <Text fontSize="xl" filter="drop-shadow(0 0 5px rgba(56, 189, 248, 0.3))">
                              {robotNames[slot.robotType].emoji}
                            </Text>
                            <Text fontSize="sm" color="gray.300" fontWeight="600">
                              {robotNames[slot.robotType].name}
                            </Text>
                          </HStack>
                        </Flex>
                        
                        <VStack align="start" spacing={2} w="full">
                          <Text color="white" fontWeight="bold" fontSize="lg">
                            {new Date(slot.date).toLocaleDateString('en-US', { 
                              weekday: 'long', 
                              month: 'long', 
                              day: 'numeric' 
                            })}
                          </Text>
                          <HStack>
                            <Box color="brand.400">🕐</Box>
                            <Text color="gray.300" fontSize="lg" fontWeight="600">
                              {slot.startTime} - {slot.endTime}
                            </Text>
                          </HStack>
                        </VStack>

                        <MotionButton
                          variant="gradient"
                          w="full"
                          size="lg"
                          isLoading={isLoading}
                          loadingText="Booking..."
                          whileHover={{ scale: 1.05 }}
                          whileTap={{ scale: 0.95 }}
                        >
                          🚀 Book This Session
                        </MotionButton>
                      </VStack>
                    </CardBody>
                  </MotionCard>
                ))}
              </MotionSimpleGrid>
            )}
          </VStack>
        </MotionBox>
      </MotionVStack>
    </Container>
  );
};

export default BookingPage;