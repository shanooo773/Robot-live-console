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
    <Box minH="100vh" bg="gradient.dark" color="gray.100">
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
