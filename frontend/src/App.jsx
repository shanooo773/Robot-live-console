import { useState } from "react";
import { Box } from "@chakra-ui/react";
import LandingPage from "./components/LandingPage";
import AuthPage from "./components/AuthPage";
import BookingPage from "./components/BookingPage";
import CodeEditor from "./components/CodeEditor";

function App() {
  const [currentPage, setCurrentPage] = useState("landing"); // landing, auth, booking, editor
  const [user, setUser] = useState(null);
  const [selectedSlot, setSelectedSlot] = useState(null);

  const handleAuth = (userData) => {
    setUser(userData);
    setCurrentPage("booking");
  };

  const handleBooking = (slot) => {
    setSelectedSlot(slot);
    setCurrentPage("editor");
  };

  const handleLogout = () => {
    setUser(null);
    setSelectedSlot(null);
    setCurrentPage("landing");
  };

  return (
    <Box 
      minH="100vh" 
      bgGradient="linear(135deg, dark.900 0%, dark.800 25%, dark.700 50%, dark.600 75%, dark.500 100%)"
      color="white"
      position="relative"
      overflow="hidden"
    >
      {/* Animated background elements */}
      <Box
        position="absolute"
        top="10%"
        left="10%"
        w="200px"
        h="200px"
        bgGradient="radial(circle, brand.500 0%, transparent 70%)"
        borderRadius="full"
        opacity="0.1"
        animation="float 6s ease-in-out infinite"
        sx={{
          "@keyframes float": {
            "0%, 100%": { transform: "translateY(0px)" },
            "50%": { transform: "translateY(-20px)" },
          },
        }}
      />
      <Box
        position="absolute"
        top="60%"
        right="15%"
        w="150px"
        h="150px"
        bgGradient="radial(circle, accent.500 0%, transparent 70%)"
        borderRadius="full"
        opacity="0.1"
        animation="float 8s ease-in-out infinite reverse"
      />
      <Box
        position="absolute"
        bottom="20%"
        left="60%"
        w="100px"
        h="100px"
        bgGradient="radial(circle, success.500 0%, transparent 70%)"
        borderRadius="full"
        opacity="0.1"
        animation="float 10s ease-in-out infinite"
      />
      
      {currentPage === "landing" && (
        <LandingPage onGetStarted={() => setCurrentPage("auth")} />
      )}
      {currentPage === "auth" && (
        <AuthPage onAuth={handleAuth} onBack={() => setCurrentPage("landing")} />
      )}
      {currentPage === "booking" && (
        <BookingPage 
          user={user} 
          onBooking={handleBooking} 
          onLogout={handleLogout}
        />
      )}
      {currentPage === "editor" && (
        <CodeEditor 
          user={user} 
          slot={selectedSlot} 
          onBack={() => setCurrentPage("booking")}
          onLogout={handleLogout}
        />
      )}
    </Box>
  );
}

export default App;
