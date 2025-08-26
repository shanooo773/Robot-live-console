import { extendTheme } from "@chakra-ui/react";

const theme = extendTheme({
  config: {
    initialColorMode: "dark",
    useSystemColorMode: false,
  },
  colors: {
    brand: {
      50: "#e3f2fd",
      100: "#bbdefb",
      200: "#90caf9",
      300: "#64b5f6",
      400: "#42a5f5",
      500: "#2196f3",
      600: "#1e88e5",
      700: "#1976d2",
      800: "#1565c0",
      900: "#0d47a1",
    },
    accent: {
      50: "#f3e5f5",
      100: "#e1bee7",
      200: "#ce93d8",
      300: "#ba68c8",
      400: "#ab47bc",
      500: "#9c27b0",
      600: "#8e24aa",
      700: "#7b1fa2",
      800: "#6a1b9a",
      900: "#4a148c",
    },
    gradient: {
      primary: "linear-gradient(135deg, #667eea 0%, #764ba2 100%)",
      secondary: "linear-gradient(135deg, #f093fb 0%, #f5576c 100%)",
      accent: "linear-gradient(135deg, #4facfe 0%, #00f2fe 100%)",
      dark: "linear-gradient(135deg, #232526 0%, #414345 100%)",
      purple: "linear-gradient(135deg, #667eea 0%, #764ba2 100%)",
      blue: "linear-gradient(135deg, #74b9ff 0%, #0984e3 100%)",
      green: "linear-gradient(135deg, #00b894 0%, #00cec9 100%)",
    },
  },
  fonts: {
    heading: `"Inter", -apple-system, BlinkMacSystemFont, "Segoe UI", Helvetica, Arial, sans-serif`,
    body: `"Inter", -apple-system, BlinkMacSystemFont, "Segoe UI", Helvetica, Arial, sans-serif`,
  },
  fontSizes: {
    xs: "0.75rem",
    sm: "0.875rem", 
    md: "1rem",
    lg: "1.125rem",
    xl: "1.25rem",
    "2xl": "1.5rem",
    "3xl": "1.875rem",
    "4xl": "2.25rem",
    "5xl": "3rem",
    "6xl": "3.75rem",
    "7xl": "4.5rem",
    "8xl": "6rem",
    "9xl": "8rem",
  },
  components: {
    Button: {
      baseStyle: {
        fontWeight: "500",
        borderRadius: "xl",
        transition: "all 0.2s ease-in-out",
        _focus: {
          boxShadow: "0 0 0 3px rgba(66, 153, 225, 0.6)",
        },
      },
      variants: {
        solid: (props) => ({
          bg: props.colorScheme === "blue" ? "gradient.blue" : undefined,
          color: "white",
          _hover: {
            transform: "translateY(-2px)",
            boxShadow: "0 10px 25px rgba(0, 0, 0, 0.2)",
            _disabled: {
              transform: "none",
            },
          },
          _active: {
            transform: "translateY(0)",
          },
        }),
        gradient: {
          background: "gradient.primary",
          color: "white",
          _hover: {
            transform: "translateY(-2px)",
            boxShadow: "0 10px 25px rgba(0, 0, 0, 0.2)",
            filter: "brightness(1.1)",
          },
          _active: {
            transform: "translateY(0)",
          },
        },
        glassmorphism: {
          background: "rgba(255, 255, 255, 0.1)",
          backdropFilter: "blur(10px)",
          border: "1px solid rgba(255, 255, 255, 0.2)",
          color: "white",
          _hover: {
            background: "rgba(255, 255, 255, 0.2)",
            transform: "translateY(-2px)",
            boxShadow: "0 10px 25px rgba(0, 0, 0, 0.2)",
          },
        },
      },
    },
    Card: {
      baseStyle: {
        container: {
          borderRadius: "2xl",
          overflow: "hidden",
          transition: "all 0.3s ease-in-out",
        },
      },
      variants: {
        glassmorphism: {
          container: {
            background: "rgba(255, 255, 255, 0.05)",
            backdropFilter: "blur(10px)",
            border: "1px solid rgba(255, 255, 255, 0.1)",
            _hover: {
              background: "rgba(255, 255, 255, 0.08)",
              transform: "translateY(-4px)",
              boxShadow: "0 20px 40px rgba(0, 0, 0, 0.3)",
            },
          },
        },
        elevated: {
          container: {
            boxShadow: "0 10px 25px rgba(0, 0, 0, 0.2)",
            _hover: {
              transform: "translateY(-4px)",
              boxShadow: "0 20px 40px rgba(0, 0, 0, 0.3)",
            },
          },
        },
      },
    },
    Input: {
      variants: {
        glassmorphism: {
          field: {
            background: "rgba(255, 255, 255, 0.05)",
            backdropFilter: "blur(10px)",
            border: "1px solid rgba(255, 255, 255, 0.2)",
            borderRadius: "xl",
            color: "white",
            _placeholder: {
              color: "whiteAlpha.600",
            },
            _hover: {
              background: "rgba(255, 255, 255, 0.08)",
              borderColor: "rgba(255, 255, 255, 0.3)",
            },
            _focus: {
              background: "rgba(255, 255, 255, 0.1)",
              borderColor: "blue.400",
              boxShadow: "0 0 0 1px rgba(66, 153, 225, 0.6)",
            },
          },
        },
      },
    },
    Badge: {
      variants: {
        gradient: {
          background: "gradient.accent",
          color: "white",
          borderRadius: "full",
          px: 3,
          py: 1,
        },
        glassmorphism: {
          background: "rgba(255, 255, 255, 0.1)",
          backdropFilter: "blur(10px)",
          border: "1px solid rgba(255, 255, 255, 0.2)",
          color: "white",
          borderRadius: "full",
        },
      },
    },
  },
  styles: {
    global: {
      body: {
        bg: "#0a0a0f",
        background: "linear-gradient(135deg, #0a0a0f 0%, #1a1a2e 100%)",
        minHeight: "100vh",
      },
      "*": {
        scrollBehavior: "smooth",
      },
    },
  },
});

export default theme;
