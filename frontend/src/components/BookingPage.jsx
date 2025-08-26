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
  Icon,
  keyframes,
} from "@chakra-ui/react";
import { useState, useEffect } from "react";
import { motion } from "framer-motion";
import { FiClock, FiCalendar, FiUser, FiCheck } from "react-icons/fi";

const MotionBox = motion(Box);
const MotionCard = motion(Card);
const MotionVStack = motion(VStack);

// Animation keyframes
const pulse = keyframes`
  0% { box-shadow: 0 0 0 0 rgba(67, 233, 123, 0.4); }
  70% { box-shadow: 0 0 0 10px rgba(67, 233, 123, 0); }
  100% { box-shadow: 0 0 0 0 rgba(67, 233, 123, 0); }
`;

const fadeInUp = {
  hidden: { opacity: 0, y: 30 },
  visible: { 
    opacity: 1, 
    y: 0,
    transition: { duration: 0.6, ease: "easeOut" }
  }
};

const staggerContainer = {
  hidden: { opacity: 0 },
  visible: {
    opacity: 1,
    transition: {
      staggerChildren: 0.1,
      delayChildren: 0.2
    }
  }
};

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
        initial="hidden"
        animate="visible"
        variants={staggerContainer}
      >
        {/* Enhanced Header */}
        <MotionBox w="full" variants={fadeInUp}>
          <HStack w="full" justify="space-between">
            <VStack align="start" spacing={3}>
              <Text 
                fontSize={{ base: "2xl", md: "3xl" }} 
                fontWeight="bold" 
                bgGradient="linear(to-r, #43e97b, #38f9d7)"
                bgClip="text"
              >
                Book Your Robot Session
              </Text>
              <HStack spacing={3}>
                <Avatar 
                  size="sm" 
                  name={user.name} 
                  bg="linear-gradient(135deg, #667eea 0%, #764ba2 100%)"
                />
                <VStack align="start" spacing={0}>
                  <Text color="white" fontWeight="medium">Welcome, {user.name}</Text>
                  <Text color="gray.400" fontSize="sm">Choose your programming session</Text>
                </VStack>
              </HStack>
            </VStack>
            <Button 
              variant="glass" 
              onClick={onLogout}
              size="lg"
            >
              Logout
            </Button>
          </HStack>
        </MotionBox>

        {/* Enhanced Filters */}
        <MotionCard w="full" variant="floating" variants={fadeInUp}>
          <CardHeader>
            <HStack>
              <Icon as={FiCalendar} color="blue.300" boxSize={5} />
              <Text 
                fontSize="lg" 
                fontWeight="bold" 
                bgGradient="linear(to-r, #4facfe, #00f2fe)"
                bgClip="text"
              >
                Filter Available Sessions
              </Text>
            </HStack>
          </CardHeader>
          <CardBody>
            <SimpleGrid columns={{ base: 1, md: 2 }} spacing={6}>
              <FormControl>
                <FormLabel color="gray.300" fontWeight="semibold">
                  <Icon as={FiCalendar} mr={2} />
                  Date
                </FormLabel>
                <Select
                  value={selectedDate}
                  onChange={(e) => setSelectedDate(e.target.value)}
                  variant="glass"
                  size="lg"
                >
                  <option value="">All dates</option>
                  {getDateOptions().map(option => (
                    <option key={option.value} value={option.value}>
                      {option.label}
                    </option>
                  ))}
                </Select>
              </FormControl>

              <FormControl>
                <FormLabel color="gray.300" fontWeight="semibold">
                  <Icon as={FiUser} mr={2} />
                  Robot Type
                </FormLabel>
                <Select
                  value={selectedRobot}
                  onChange={(e) => setSelectedRobot(e.target.value)}
                  variant="glass"
                  size="lg"
                >
                  <option value="">All robots</option>
                  <option value="turtlebot">🤖 TurtleBot3</option>
                  <option value="arm">🦾 Robot Arm</option>
                  <option value="hand">🤲 Robot Hand</option>
                </Select>
              </FormControl>
            </SimpleGrid>
          </CardBody>
        </MotionCard>

        {/* Enhanced Available Slots */}
        <MotionVStack w="full" spacing={6} variants={fadeInUp}>
          <HStack w="full" justify="space-between">
            <HStack>
              <Icon as={FiCheck} color="green.300" boxSize={6} />
              <Text 
                fontSize="xl" 
                fontWeight="bold" 
                color="white"
              >
                Available Sessions ({availableSlots.length})
              </Text>
            </HStack>
            <Badge 
              variant="glass"
              bg="rgba(67, 233, 123, 0.1)"
              border="1px solid rgba(67, 233, 123, 0.3)"
              color="green.300"
              px={4} 
              py={2}
              borderRadius="full"
              animation={`${pulse} 2s infinite`}
            >
              {availableSlots.length} slots free
            </Badge>
          </HStack>

          {availableSlots.length === 0 ? (
            <MotionCard 
              w="full" 
              variant="glass"
              initial={{ opacity: 0, scale: 0.9 }}
              animate={{ opacity: 1, scale: 1 }}
              transition={{ duration: 0.5 }}
            >
              <CardBody textAlign="center" py={16}>
                <Text fontSize="4xl" mb={4}>🤖</Text>
                <Text fontSize="xl" color="gray.300" mb={2}>
                  No available slots found
                </Text>
                <Text color="gray.500">
                  Try adjusting your filters or check back later
                </Text>
              </CardBody>
            </MotionCard>
          ) : (
            <SimpleGrid columns={{ base: 1, md: 2, lg: 3 }} spacing={6} w="full">
              {availableSlots.map((slot, index) => (
                <MotionCard 
                  key={slot.id}
                  variant="floating"
                  cursor="pointer"
                  initial={{ opacity: 0, y: 30 }}
                  animate={{ opacity: 1, y: 0 }}
                  transition={{ delay: index * 0.1, duration: 0.5 }}
                  whileHover={{ 
                    scale: 1.02,
                    transition: { duration: 0.2 }
                  }}
                  whileTap={{ scale: 0.98 }}
                >
                  <CardBody p={6}>
                    <VStack spacing={4} align="start">
                      <HStack justify="space-between" w="full">
                        <Badge 
                          variant="glass"
                          bg="rgba(67, 233, 123, 0.1)"
                          border="1px solid rgba(67, 233, 123, 0.3)"
                          color="green.300"
                          borderRadius="full"
                        >
                          <Icon as={FiCheck} mr={1} />
                          Available
                        </Badge>
                        <HStack>
                          <Text fontSize="2xl">{robotNames[slot.robotType].emoji}</Text>
                          <Text fontSize="sm" color="gray.300" fontWeight="medium">
                            {robotNames[slot.robotType].name}
                          </Text>
                        </HStack>
                      </HStack>
                      
                      <VStack align="start" spacing={2} w="full">
                        <HStack color="white">
                          <Icon as={FiCalendar} color="blue.300" />
                          <Text fontWeight="bold">
                            {new Date(slot.date).toLocaleDateString('en-US', { 
                              weekday: 'long', 
                              month: 'long', 
                              day: 'numeric' 
                            })}
                          </Text>
                        </HStack>
                        <HStack color="gray.300">
                          <Icon as={FiClock} color="purple.300" />
                          <Text fontSize="lg" fontWeight="medium">
                            {slot.startTime} - {slot.endTime}
                          </Text>
                        </HStack>
                      </VStack>

                      <Button
                        variant="success"
                        w="full"
                        size="lg"
                        onClick={() => handleBookSlot(slot)}
                        isLoading={isLoading}
                        loadingText="Booking..."
                        borderRadius="xl"
                      >
                        Book This Session
                      </Button>
                    </VStack>
                  </CardBody>
                </MotionCard>
              ))}
            </SimpleGrid>
          )}
        </MotionVStack>

        {/* Enhanced Unavailable Sessions */}
        {bookedSlots.length > 0 && (
          <MotionVStack w="full" spacing={6} variants={fadeInUp}>
            <Box w="full">
              <Divider 
                borderColor="rgba(255, 255, 255, 0.1)" 
                borderWidth="1px"
                background="linear-gradient(90deg, transparent, rgba(255, 255, 255, 0.2), transparent)"
              />
            </Box>
            
            <HStack w="full" justify="space-between">
              <HStack>
                <Icon as={FiClock} color="red.300" boxSize={6} />
                <Text fontSize="xl" fontWeight="bold" color="white">
                  Unavailable Sessions
                </Text>
              </HStack>
              <Badge 
                variant="glass"
                bg="rgba(245, 87, 108, 0.1)"
                border="1px solid rgba(245, 87, 108, 0.3)"
                color="red.300"
                px={4} 
                py={2}
                borderRadius="full"
              >
                {bookedSlots.length} slots taken
              </Badge>
            </HStack>

            <SimpleGrid columns={{ base: 1, md: 2, lg: 3 }} spacing={6} w="full">
              {bookedSlots.slice(0, 6).map((slot, index) => (
                <MotionCard
                  key={slot.id}
                  variant="glass"
                  opacity={0.6}
                  initial={{ opacity: 0, y: 30 }}
                  animate={{ opacity: 0.6, y: 0 }}
                  transition={{ delay: index * 0.1, duration: 0.5 }}
                  position="relative"
                  overflow="hidden"
                  _before={{
                    content: '""',
                    position: 'absolute',
                    top: 0,
                    left: 0,
                    right: 0,
                    bottom: 0,
                    bg: 'linear-gradient(45deg, transparent 49%, rgba(245, 87, 108, 0.1) 50%, transparent 51%)',
                    pointerEvents: 'none',
                  }}
                >
                  <CardBody p={6}>
                    <VStack spacing={4} align="start">
                      <HStack justify="space-between" w="full">
                        <Badge 
                          variant="glass"
                          bg="rgba(245, 87, 108, 0.1)"
                          border="1px solid rgba(245, 87, 108, 0.3)"
                          color="red.300"
                          borderRadius="full"
                        >
                          Taken
                        </Badge>
                        <HStack>
                          <Text fontSize="2xl">{robotNames[slot.robotType].emoji}</Text>
                          <Text fontSize="sm" color="gray.400" fontWeight="medium">
                            {robotNames[slot.robotType].name}
                          </Text>
                        </HStack>
                      </HStack>
                      
                      <VStack align="start" spacing={2} w="full">
                        <HStack color="gray.300">
                          <Icon as={FiCalendar} color="gray.500" />
                          <Text>
                            {new Date(slot.date).toLocaleDateString('en-US', { 
                              weekday: 'long', 
                              month: 'long', 
                              day: 'numeric' 
                            })}
                          </Text>
                        </HStack>
                        <HStack color="gray.400">
                          <Icon as={FiClock} color="gray.500" />
                          <Text>
                            {slot.startTime} - {slot.endTime}
                          </Text>
                        </HStack>
                      </VStack>

                      <Text fontSize="sm" color="gray.500" fontStyle="italic">
                        Booked by {slot.bookedBy}
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