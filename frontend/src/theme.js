import { extendTheme } from "@chakra-ui/react";

const theme = extendTheme({
  config: {
    initialColorMode: "dark",
    useSystemColorMode: false,
  },
  fonts: {
    heading: "'Inter', -apple-system, BlinkMacSystemFont, sans-serif",
    body: "'Inter', -apple-system, BlinkMacSystemFont, sans-serif",
  },
  colors: {
    brand: {
      50: '#e6fffa',
      100: '#b3f3eb',
      200: '#81e6d9',
      300: '#4fd1c7',
      400: '#38b2ac',
      500: '#319795',
      600: '#2c7a7b',
      700: '#285e61',
      800: '#234e52',
      900: '#1d4044',
    },
    accent: {
      50: '#fef5e7',
      100: '#fce2c3',
      200: '#f9cc94',
      300: '#f6b465',
      400: '#f39d36',
      500: '#f08500',
      600: '#d47300',
      700: '#b86100',
      800: '#9c4f00',
      900: '#803d00',
    },
    gradient: {
      primary: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
      secondary: 'linear-gradient(135deg, #f093fb 0%, #f5576c 100%)',
      success: 'linear-gradient(135deg, #4facfe 0%, #00f2fe 100%)',
      accent: 'linear-gradient(135deg, #43e97b 0%, #38f9d7 100%)',
      dark: 'linear-gradient(135deg, #0c0c0c 0%, #1a1a2e 50%, #16213e 100%)',
      card: 'linear-gradient(145deg, rgba(255,255,255,0.1) 0%, rgba(255,255,255,0.05) 100%)',
    }
  },
  components: {
    Button: {
      baseStyle: {
        fontWeight: 'semibold',
        borderRadius: 'lg',
        transition: 'all 0.3s ease',
        _hover: {
          transform: 'translateY(-2px)',
          boxShadow: 'lg',
        },
        _active: {
          transform: 'translateY(0px)',
        }
      },
      variants: {
        gradient: {
          background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
          color: 'white',
          _hover: {
            background: 'linear-gradient(135deg, #5a6fd8 0%, #6a4190 100%)',
            transform: 'translateY(-2px)',
            boxShadow: '0 10px 25px rgba(102, 126, 234, 0.4)',
          },
          _active: {
            transform: 'translateY(0px)',
          }
        },
        success: {
          background: 'linear-gradient(135deg, #4facfe 0%, #00f2fe 100%)',
          color: 'white',
          _hover: {
            background: 'linear-gradient(135deg, #3d8bfe 0%, #00d9fe 100%)',
            transform: 'translateY(-2px)',
            boxShadow: '0 10px 25px rgba(79, 172, 254, 0.4)',
          }
        },
        accent: {
          background: 'linear-gradient(135deg, #43e97b 0%, #38f9d7 100%)',
          color: 'white',
          _hover: {
            background: 'linear-gradient(135deg, #36d969 0%, #2de6c5 100%)',
            transform: 'translateY(-2px)',
            boxShadow: '0 10px 25px rgba(67, 233, 123, 0.4)',
          }
        },
        glass: {
          background: 'rgba(255, 255, 255, 0.1)',
          backdropFilter: 'blur(10px)',
          border: '1px solid rgba(255, 255, 255, 0.2)',
          color: 'white',
          _hover: {
            background: 'rgba(255, 255, 255, 0.15)',
            transform: 'translateY(-2px)',
            boxShadow: '0 10px 25px rgba(0, 0, 0, 0.2)',
          }
        }
      }
    },
    Card: {
      baseStyle: {
        container: {
          borderRadius: 'xl',
          transition: 'all 0.3s ease',
          _hover: {
            transform: 'translateY(-4px)',
            boxShadow: '0 20px 40px rgba(0, 0, 0, 0.3)',
          }
        }
      },
      variants: {
        glass: {
          container: {
            background: 'rgba(255, 255, 255, 0.05)',
            backdropFilter: 'blur(20px)',
            border: '1px solid rgba(255, 255, 255, 0.1)',
            boxShadow: '0 8px 32px rgba(0, 0, 0, 0.3)',
          }
        },
        gradient: {
          container: {
            background: 'linear-gradient(145deg, rgba(255,255,255,0.1) 0%, rgba(255,255,255,0.05) 100%)',
            border: '1px solid rgba(255, 255, 255, 0.1)',
            boxShadow: '0 8px 32px rgba(0, 0, 0, 0.3)',
          }
        },
        floating: {
          container: {
            background: 'rgba(255, 255, 255, 0.08)',
            backdropFilter: 'blur(15px)',
            border: '1px solid rgba(255, 255, 255, 0.15)',
            boxShadow: '0 15px 35px rgba(0, 0, 0, 0.4)',
            transition: 'all 0.4s cubic-bezier(0.4, 0, 0.2, 1)',
            _hover: {
              transform: 'translateY(-8px) scale(1.02)',
              boxShadow: '0 25px 50px rgba(0, 0, 0, 0.5)',
              border: '1px solid rgba(255, 255, 255, 0.25)',
            }
          }
        }
      }
    },
    Badge: {
      baseStyle: {
        borderRadius: 'full',
        fontWeight: 'semibold',
        fontSize: 'xs',
        px: 3,
        py: 1,
      },
      variants: {
        gradient: {
          background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
          color: 'white',
        },
        glass: {
          background: 'rgba(255, 255, 255, 0.1)',
          backdropFilter: 'blur(10px)',
          border: '1px solid rgba(255, 255, 255, 0.2)',
          color: 'white',
        }
      }
    },
    Input: {
      variants: {
        glass: {
          field: {
            background: 'rgba(255, 255, 255, 0.05)',
            backdropFilter: 'blur(10px)',
            border: '1px solid rgba(255, 255, 255, 0.1)',
            borderRadius: 'lg',
            color: 'white',
            _placeholder: {
              color: 'rgba(255, 255, 255, 0.5)',
            },
            _hover: {
              border: '1px solid rgba(255, 255, 255, 0.2)',
            },
            _focus: {
              border: '1px solid rgba(102, 126, 234, 0.5)',
              boxShadow: '0 0 0 1px rgba(102, 126, 234, 0.3)',
            }
          }
        }
      }
    },
    Select: {
      variants: {
        glass: {
          field: {
            background: 'rgba(255, 255, 255, 0.05)',
            backdropFilter: 'blur(10px)',
            border: '1px solid rgba(255, 255, 255, 0.1)',
            borderRadius: 'lg',
            color: 'white',
            _hover: {
              border: '1px solid rgba(255, 255, 255, 0.2)',
            },
            _focus: {
              border: '1px solid rgba(102, 126, 234, 0.5)',
              boxShadow: '0 0 0 1px rgba(102, 126, 234, 0.3)',
            }
          }
        }
      }
    }
  },
  styles: {
    global: {
      body: {
        background: 'linear-gradient(135deg, #0c0c0c 0%, #1a1a2e 50%, #16213e 100%)',
        backgroundAttachment: 'fixed',
      },
      '*': {
        scrollBehavior: 'smooth',
      }
    }
  }
});

export default theme;
